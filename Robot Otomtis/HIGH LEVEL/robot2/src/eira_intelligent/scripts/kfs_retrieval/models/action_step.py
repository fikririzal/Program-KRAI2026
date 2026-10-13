from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class ActionStep:
    step: int
    node: int
    action: str
    elevation: str
    direction: str
    heading: int
    target: Any
    cost: float