"""GPU-side NaVILA multipart inference service.

Run this file in the NaVILA model environment. It deliberately accepts image
bytes rather than paths from the NX host.
"""
import hashlib
import json
import os
import time
from pathlib import Path
from typing import List

from fastapi import FastAPI, File, HTTPException, UploadFile, Form

MODEL_PATH = os.getenv("NAVILA_MODEL_PATH", "")
DEVICE = os.getenv("NAVILA_DEVICE", "cuda:0")
MAX_BYTES = int(os.getenv("NAVILA_MAX_FRAME_BYTES", str(8 * 1024 * 1024)))
app = FastAPI(title="ELF-OS NaVILA model deployment", version="1.0.0")
state = {}


def load_model():
    if not MODEL_PATH:
        raise RuntimeError("NAVILA_MODEL_PATH is required")
    import torch
    from llava.conversation import conv_templates
    from llava.mm_utils import get_model_name_from_path
    from llava.model.builder import load_pretrained_model
    name = get_model_name_from_path(MODEL_PATH)
    tokenizer, model, processor, context_len = load_pretrained_model(MODEL_PATH, name, None, device=DEVICE)
    model.eval()
    state.update(tokenizer=tokenizer, model=model, processor=processor, context_len=context_len,
                 conv=conv_templates["llama_3"].copy(), model_path=MODEL_PATH)


@app.on_event("startup")
def startup():
    load_model()


@app.get("/health")
def health():
    return {"status": "ready" if state else "loading", "model_path": MODEL_PATH, "device": DEVICE}


@app.post("/v1/navigate")
async def navigate(metadata: str = Form(...), frames: List[UploadFile] = File(...)):
    try:
        request = json.loads(metadata)
    except (TypeError, ValueError) as exc:
        raise HTTPException(400, "metadata must be JSON") from exc
    if request.get("schema") != "elf.navigate-request.v1" or not request.get("request_id"):
        raise HTTPException(400, "invalid request schema")
    expected = request.get("frames")
    if not isinstance(expected, list) or len(expected) != len(frames) or request.get("num_video_frames") != len(frames):
        raise HTTPException(400, "frame count mismatch")
    import tempfile
    from PIL import Image
    with tempfile.TemporaryDirectory(prefix="elf-navila-") as directory:
        paths = []
        for index, upload in enumerate(frames):
            data = await upload.read()
            if len(data) > MAX_BYTES or hashlib.sha256(data).hexdigest() != expected[index].get("sha256"):
                raise HTTPException(400, "frame hash or size mismatch")
            path = Path(directory) / ("frame_%03d.jpg" % index)
            path.write_bytes(data)
            try:
                with Image.open(str(path)) as image:
                    image.verify()
            except Exception as exc:
                raise HTTPException(400, "invalid JPEG frame") from exc
            paths.append(path)
        if not state:
            raise HTTPException(503, "model is not loaded")
        started = time.monotonic()
        action = _infer(request["instruction"], paths)
        return {"request_id": request["request_id"], "action": action,
                "model_path": state["model_path"], "inference_latency_s": time.monotonic() - started}


def _infer(instruction, paths):
    import torch
    from PIL import Image
    from llava.constants import IMAGE_TOKEN_INDEX
    from llava.conversation import SeparatorStyle
    from llava.mm_utils import KeywordsStoppingCriteria, process_images, tokenizer_image_token
    images = [Image.open(str(path)).convert("RGB") for path in paths]
    image_token = "<image>\n"
    question = ("Imagine you are a robot programmed for navigation tasks. You have been given a video of "
                "historical observations " + image_token * (len(images) - 1) + "and current observation <image>. "
                "Your assigned task is: \"%s\" Analyze the observations and output exactly one action: "
                "move forward 25/50/75 cm, turn left/right 15/30/45 degrees, or stop." % instruction)
    conv = state["conv"].copy()
    conv.append_message(conv.roles[0], question)
    conv.append_message(conv.roles[1], None)
    prompt = conv.get_prompt()
    tokenizer = state["tokenizer"]
    inputs = tokenizer_image_token(prompt, tokenizer, IMAGE_TOKEN_INDEX, return_tensors="pt").unsqueeze(0).to(state["model"].device)
    tensor = process_images(images, state["processor"], state["model"].config).to(state["model"].device, dtype=torch.float16)
    with torch.inference_mode():
        output = state["model"].generate(inputs, images=tensor, do_sample=False, max_new_tokens=64,
                                          use_cache=True, pad_token_id=tokenizer.eos_token_id)
    return tokenizer.batch_decode(output, skip_special_tokens=True)[0].strip()


if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host=os.getenv("NAVILA_HOST", "127.0.0.1"), port=int(os.getenv("NAVILA_PORT", "8011")))
