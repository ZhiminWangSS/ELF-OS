import re
from dataclasses import dataclass


FORWARD_CM = (25, 50, 75)
TURN_DEG = (15, 30, 45)


@dataclass(frozen=True)
class DiscreteAction:
    name: str
    value: int = 0
    unit: str = "none"
    raw: str = ""

    @property
    def is_stop(self):
        return self.name == "stop"


def parse_action(raw: str) -> DiscreteAction:
    text = (raw or "").strip()
    lowered = text.lower()
    candidates = []
    if re.search(r"\bstop\b|\bcompleted?\b|\barrived\b", lowered):
        candidates.append(DiscreteAction("stop", raw=text))
    for name, pattern, allowed, unit in (
        ("forward", r"\b(?:move\s+)?forward\s+(\d+)\s*(?:cm|centimeters?)\b", FORWARD_CM, "cm"),
        ("turn_left", r"\bturn\s+left\s+(\d+)\s*(?:deg|degrees?)\b", TURN_DEG, "degree"),
        ("turn_right", r"\bturn\s+right\s+(\d+)\s*(?:deg|degrees?)\b", TURN_DEG, "degree"),
    ):
        match = re.search(pattern, lowered)
        if match:
            value = int(match.group(1))
            candidates.append(DiscreteAction(name, value if value in allowed else -1, unit, text))
    if len(candidates) != 1 or candidates[0].value < 0:
        return DiscreteAction("stop", raw=text)
    return candidates[0]


def is_explicit_stop(raw: str) -> bool:
    return bool(re.search(r"\bstop\b|\bcompleted?\b|\barrived\b", (raw or "").lower()))
