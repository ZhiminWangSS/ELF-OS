import copy
import importlib.util
import time
from pathlib import Path


DEFAULT_MODEL_PATH = "/mnt/storage2/users/zmwang/SeekVLN/model_exports/20260819_SeekVLN-Full-SFT-V2-3-4K-20260817-180914_global_step_9261_hf_full"


def _load_auxthink_base():
    """Load the projector helper without importing llava.trl's trainer stack."""
    import llava
    helper = Path(llava.__file__).resolve().parent / "trl" / "models" / "modeling_auxthink_projector.py"
    spec = importlib.util.spec_from_file_location("seekvln_auxthink_projector", str(helper))
    if spec is None or spec.loader is None:
        raise ImportError("cannot load AuxThink projector helper")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module.load_auxthink_base


class SeekVLNModel:
    def __init__(self, model_path=None, device="cuda:0"):
        import torch
        from llava.constants import IMAGE_TOKEN_INDEX
        from llava.mm_utils import get_model_name_from_path, process_images, tokenizer_image_token
        from llava.model.builder import load_pretrained_model
        load_auxthink_base = _load_auxthink_base()
        from llava.verl.seekvln_inference import SEEK_ENVIRONMENT_TURN, build_seekvln_stage1_prompt, parse_seekvln_response, tokenize_seekvln_conversation
        from llava.verl.seekvln_tokens import ensure_seekvln_tokenizer
        self.torch = torch; self.IMAGE_TOKEN_INDEX = IMAGE_TOKEN_INDEX
        self.process_images = process_images; self.tokenizer_image_token = tokenizer_image_token
        self.seek_environment_turn = SEEK_ENVIRONMENT_TURN
        self.build_prompt = build_seekvln_stage1_prompt
        self.parse_response = parse_seekvln_response
        self.tokenize = tokenize_seekvln_conversation
        model_path = str(Path(model_path or DEFAULT_MODEL_PATH).expanduser())
        if not Path(model_path).is_dir():
            raise FileNotFoundError("SeekVLN checkpoint does not exist: " + model_path)
        tokenizer, model, processor, _ = load_auxthink_base(
            load_pretrained_model, model_path, get_model_name_from_path(model_path))
        self.control_tokens = ensure_seekvln_tokenizer(tokenizer)
        model.to(device=device, dtype=torch.float16); model.eval()
        self.tokenizer, self.model, self.processor = tokenizer, model, processor
        self.device = torch.device(device)
        self.control_ids = {name: int(tokenizer.convert_tokens_to_ids(name))
                            for name in ("<nav>", "<seek>", "<think>", "</think>")}
        self.model_version = Path(model_path).name

    def _inputs(self, prompt_ids, images):
        query = self.torch.tensor(prompt_ids, dtype=self.torch.long, device=self.device).unsqueeze(0)
        image_tensor = self.process_images(images, self.processor, self.model.config)
        image_tensor = image_tensor.to(device=self.device, dtype=self.torch.float16)
        return query, image_tensor

    def _generate(self, prompt_ids, images):
        query, image_tensor = self._inputs(prompt_ids, images)
        from transformers import GenerationConfig
        config = copy.deepcopy(getattr(self.model, "generation_config", None) or GenerationConfig())
        config.max_new_tokens = 160; config.do_sample = False; config.temperature = 0.0
        config.top_p = 1.0; config.top_k = -1
        config.pad_token_id = config.pad_token_id or self.tokenizer.eos_token_id
        with self.torch.inference_mode():
            output = self.model.generate(query, images=image_tensor,
                                         attention_mask=self.torch.ones_like(query),
                                         generation_config=config, use_cache=True)
        sequence = output[0]
        if sequence.shape[0] >= query.shape[1] and self.torch.equal(sequence[: query.shape[1]], query[0]):
            sequence = sequence[query.shape[1]:]
        ids = [int(item) for item in sequence.tolist()]
        if ids and self.tokenizer.eos_token_id is not None and ids[-1] == int(self.tokenizer.eos_token_id):
            ids.pop()
        return self.tokenizer.decode(ids, skip_special_tokens=False).strip()

    def _mode(self, prompt_ids, images):
        query, image_tensor = self._inputs(prompt_ids, images)
        prepared = self.model.prepare_inputs_labels_for_multimodal(
            query, None, self.torch.ones_like(query), None, None, image_tensor)
        input_ids, position_ids, attention_mask, _, embeds, _ = prepared
        llm = self.model.get_llm()
        with self.torch.inference_mode():
            output = llm(input_ids=input_ids, attention_mask=attention_mask,
                         position_ids=position_ids, inputs_embeds=embeds,
                         use_cache=False, return_dict=True)
            # Transformers versions differ in whether the causal-LM wrapper
            # exposes ``logits`` or only the hidden state. Prefer the former.
            if getattr(output, "logits", None) is not None:
                logits = output.logits[:, -1, :][0].float()
            else:
                hidden = getattr(output, "last_hidden_state", None)
                if hidden is None:
                    hidden = output.hidden_states[-1]
                logits = llm.lm_head(hidden[:, -1, :])[0].float()
        scores = [float(logits[self.control_ids["<nav>"]]), float(logits[self.control_ids["<seek>"]])]
        return ("nav" if scores[0] >= scores[1] else "seek"), scores

    def navigate(self, phase, instruction, images, mode=None):
        started = time.monotonic()
        prompt = self.build_prompt(instruction, len(images) if mode != "seek" else len(images) - 3)
        if phase == "mode":
            prompt_ids = self.tokenize(self.tokenizer, [{"from": "human", "value": prompt}], True)
            selected, scores = self._mode(prompt_ids, images)
            return {"mode": selected, "scores": scores, "inference_s": time.monotonic() - started}
        history = images
        if mode == "nav":
            history = images
            prompt_ids = self.tokenize(self.tokenizer, [{"from": "human", "value": prompt}], True)
            text = self._generate([*prompt_ids, self.control_ids["<nav>"]], history)
            parsed = self.parse_response("nav", text)
        elif mode == "seek":
            history, views = images[:-3], images[-3:]
            conversation = [{"from": "human", "value": prompt}, {"from": "gpt", "value": "<seek>"},
                            {"from": "human", "value": self.seek_environment_turn}]
            prompt_ids = self.tokenize(self.tokenizer, conversation, True)
            text = self._generate(prompt_ids, [*history, *views])
            parsed = self.parse_response("seek", text)
        else:
            raise ValueError("action phase requires mode")
        return {"raw_text": parsed.text, "format_valid": parsed.format_valid,
                "final_action_text": parsed.final_action_text, "actions": parsed.actions,
                "invalid_reason": parsed.invalid_reason, "inference_s": time.monotonic() - started}
