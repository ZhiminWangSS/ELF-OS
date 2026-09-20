from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional


@dataclass
class SubgoalRequest:
    schema: str = "elf.subgoal-request.v1"
    subgoal_id: str = "subgoal-1"
    instruction: str = ""
    model_id: str = "seekvln-4k-sft"
    max_decisions: int = 1
    max_forward_m: float = 0.25
    max_seconds: float = 60.0

    def validate(self):
        if self.schema != "elf.subgoal-request.v1":
            raise ValueError("unsupported subgoal schema")
        if not self.subgoal_id.strip() or not self.instruction.strip():
            raise ValueError("subgoal_id and instruction must not be empty")
        if self.model_id not in ("navila", "seekvln-4k-sft"):
            raise ValueError("unsupported model_id")
        if self.max_decisions < 1 or self.max_forward_m < 0 or self.max_seconds <= 0:
            raise ValueError("invalid execution budget")


@dataclass
class SubgoalResult:
    schema: str = "elf.subgoal-result.v1"
    subgoal_id: str = ""
    model_id: str = ""
    status: str = "inference_failed"
    decisions: int = 0
    forward_m: float = 0.0
    elapsed_s: float = 0.0
    steps: List[Dict[str, Any]] = field(default_factory=list)
    error: Optional[str] = None
    error_code: Optional[str] = None
    artifacts: Optional[str] = None

    def to_dict(self):
        return self.__dict__.copy()
