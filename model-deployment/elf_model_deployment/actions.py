import re
from dataclasses import dataclass


@dataclass(frozen=True)
class PrimitiveAction:
    name: str
    value: int = 0
    unit: str = "none"
    raw: str = ""

    @property
    def is_stop(self):
        return self.name == "stop"


def parse_seekvln_output(raw: str, mode: str = "nav") -> PrimitiveAction:
    """Fail closed; unlike Habitat evaluation, never choose a random action."""
    text = str(raw or "").strip()
    if mode == "seek":
        required = ("</seek><think>", "Completed tasks:", "Next task:", "Key Evidence:", "</think><nav>")
        if not all(item in text for item in required):
            return PrimitiveAction("invalid", raw=text)
        text = text.split("</think><nav>", 1)[1]
    elif mode != "nav":
        return PrimitiveAction("invalid", raw=text)
    text = text.split(",", 1)[0].strip()
    if re.search(r"\bstop\b", text, re.I):
        if re.search(r"forward|turn\s+(left|right)", text, re.I):
            return PrimitiveAction("invalid", raw=raw)
        return PrimitiveAction("stop", raw=str(raw))
    match = re.search(r"\b(left|right|forward)\b\D*(-?\d+)", text, re.I)
    if not match:
        return PrimitiveAction("invalid", raw=str(raw))
    direction, number = match.group(1).lower(), int(match.group(2))
    if re.search(r"\b(left|right|forward)\b", text[match.end():], re.I):
        return PrimitiveAction("invalid", raw=str(raw))
    if direction == "forward":
        if number not in (25, 50, 75) or not re.search(r"cm|centimeter", text, re.I):
            return PrimitiveAction("invalid", raw=str(raw))
        return PrimitiveAction("forward", number, "cm", str(raw))
    if number not in (15, 30, 45) or not re.search(r"deg|degree", text, re.I):
        return PrimitiveAction("invalid", raw=str(raw))
    return PrimitiveAction("turn_" + direction, number, "degree", str(raw))


def parse_seekvln_prediction(prediction, mode="nav"):
    """Validate the official parser result and independently parse its text."""
    if prediction.get("format_valid") is not True:
        return PrimitiveAction("invalid", raw=str(prediction.get("raw_text", "")))
    actions = prediction.get("actions")
    if not isinstance(actions, list) or not 1 <= len(actions) <= 3:
        return PrimitiveAction("invalid", raw=str(prediction.get("raw_text", "")))
    if any(not isinstance(item, int) or isinstance(item, bool) for item in actions):
        return PrimitiveAction("invalid", raw=str(prediction.get("raw_text", "")))
    if len(set(actions)) != 1 or actions[0] not in (0, 1, 2, 3):
        return PrimitiveAction("invalid", raw=str(prediction.get("raw_text", "")))
    if actions[0] == 0 and len(actions) != 1:
        return PrimitiveAction("invalid", raw=str(prediction.get("raw_text", "")))
    value = {1: ("forward", 25 * len(actions), "cm"),
             2: ("turn_left", 15 * len(actions), "degree"),
             3: ("turn_right", 15 * len(actions), "degree"),
             0: ("stop", 0, "none")}[actions[0]]
    parsed = parse_seekvln_output(prediction.get("raw_text", ""), mode)
    if (parsed.name, parsed.value, parsed.unit) != value:
        return PrimitiveAction("invalid", raw=str(prediction.get("raw_text", "")))
    return parsed


def action_to_go2_args(action: PrimitiveAction):
    if action.name == "invalid":
        raise ValueError("invalid_model_output")
    return ["--action", action.name, "--value", str(action.value), "--unit", action.unit]
