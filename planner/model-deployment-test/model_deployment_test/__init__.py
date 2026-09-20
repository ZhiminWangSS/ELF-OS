"""NaVILA model-deployment execution module for ELF-OS."""

from .contracts import SubgoalRequest, SubgoalResult
from .runner import execute_subgoal

__all__ = ["SubgoalRequest", "SubgoalResult", "execute_subgoal"]
