from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional


@dataclass
class SubgoalRequest:
    schema: str = "elf.subgoal-request.v1"
    subgoal_id: str = "subgoal-1"
    instruction: str = ""
    max_decisions: int = 1
    max_forward_m: float = 0.25
    max_seconds: float = 60.0

    def validate(self) -> None:
        if self.schema != "elf.subgoal-request.v1":
            raise ValueError("unsupported subgoal schema")
        if not self.subgoal_id.strip() or not self.instruction.strip():
            raise ValueError("subgoal_id and instruction must not be empty")
        if self.max_decisions < 1 or self.max_forward_m < 0 or self.max_seconds <= 0:
            raise ValueError("invalid execution budget")


@dataclass
class SubgoalResult:
    schema: str = "elf.subgoal-result.v1"
    subgoal_id: str = ""
    status: str = "inference_failed"
    decisions: int = 0
    forward_m: float = 0.0
    elapsed_s: float = 0.0
    steps: List[Dict[str, Any]] = field(default_factory=list)
    error: Optional[str] = None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "schema": self.schema,
            "subgoal_id": self.subgoal_id,
            "status": self.status,
            "decisions": self.decisions,
            "forward_m": self.forward_m,
            "elapsed_s": self.elapsed_s,
            "steps": self.steps,
            "error": self.error,
        }
