#!/usr/bin/env python3
import json
import os
import time

from fastapi import FastAPI, HTTPException, Request
from fastapi.responses import JSONResponse

from model_runner import SeekVLNModel
from protocol import parse_metadata, validate_request


app = FastAPI(title="ELF-OS SeekVLN deployment", version="1.0.0")
MODEL = None


@app.on_event("startup")
def load_model():
    global MODEL
    MODEL = SeekVLNModel(os.environ.get("SEEKVLN_MODEL_PATH"), os.environ.get("SEEKVLN_DEVICE", "cuda:0"))


@app.get("/healthz")
def healthz():
    return {"status": "ok", "model_version": MODEL.model_version if MODEL else None}


@app.post("/v1/navigate")
async def navigate(request: Request):
    try:
        form = await request.form()
        metadata = parse_metadata(form.get("metadata"))
        parts = {}
        for key, value in form.multi_items():
            if key.startswith("frame_"):
                parts[key] = await value.read()
        names = validate_request(metadata, parts)
        from PIL import Image
        import io
        images = [Image.open(io.BytesIO(parts[name])).convert("RGB") for name in names]
        result = MODEL.navigate(metadata["phase"], metadata["instruction"], images, metadata.get("mode"))
        result.update({"schema": "elf.seekvln-response.v1", "request_id": metadata["request_id"],
                       "phase": metadata["phase"], "model_version": MODEL.model_version})
        return JSONResponse(result)
    except Exception as exc:
        raise HTTPException(status_code=400, detail=str(exc))
