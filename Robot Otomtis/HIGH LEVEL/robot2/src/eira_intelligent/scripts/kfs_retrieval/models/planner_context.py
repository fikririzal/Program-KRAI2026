from dataclasses import dataclass, field


@dataclass
class PlannerContext:
    team: str = "blue"
    r2_pos: int = 0
    r2_heading: int = 0

    r2_targets: list[int] = field(default_factory=list)
    r1_targets: list[int] = field(default_factory=list)
    fake_walls: list[int] = field(default_factory=list)
    dynamic_obstacles: list[int] = field(default_factory=list)

    r1_pos: int = 0
    inventory: list[int] = field(default_factory=list)
    validated_empty: list[int] = field(default_factory=list)
    visited_history: list[int] = field(default_factory=list)

    retrieval_mode: str = "auto"