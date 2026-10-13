from .route_evaluator import RouteEvaluator
from .state_space_evaluator import StateSpaceEvaluator


# EvaluatorClass = RouteEvaluator
EvaluatorClass = StateSpaceEvaluator


class R2Planner:
    """
    R2 planner wrapper.
    - RouteEvaluator       : target-order/permutation based
    - StateSpaceEvaluator  : priority queue total_cost based
    """

    def __init__(self, search_engine):
        self.search_engine = search_engine
        self.evaluator = EvaluatorClass(
            search_engine=search_engine,
            max_capacity=4,
        )

    def plan(
        self,
        team,
        r2_pos,
        r2_heading,
        r2_targets,
        r1_targets,
        fake_walls,
        dynamic_obstacles=None,
        inventory=None,
        validated_empty=None,
        visited_history=None,
        retrieval_mode="auto",
    ):
        dynamic_obstacles = dynamic_obstacles or []
        inventory = inventory or []
        validated_empty = validated_empty or []
        visited_history = visited_history or []

        clean_r2 = [
            target
            for target in r2_targets
            if target not in inventory and target not in validated_empty
        ]

        clean_r1 = [
            target
            for target in r1_targets
            if target not in validated_empty
        ]

        clean_dynamic_obstacles = [
            obstacle
            for obstacle in dynamic_obstacles
            if obstacle not in validated_empty
        ]

        all_obstacles = set(fake_walls) | set(clean_dynamic_obstacles)
        current_payload = len(inventory)

        result = self.evaluator.generate(
            current_node=r2_pos,
            current_heading=r2_heading,
            r2_targets=clean_r2,
            r1_kfs=clean_r1,
            fake_walls=all_obstacles,
            dynamic_obstacles=[],
            team=team,
            current_payload=current_payload,
            visited_history=visited_history,
            retrieval_mode=retrieval_mode,
        )

        evaluator_name = self.evaluator.__class__.__name__

        return {
            "planner": f"{evaluator_name}/{self.search_engine.name}",
            "r2_plan": result["plan"],
            "r2_total_cost": result["total_cost"],
            "r1_rec": [],
            "clean_r1_targets": clean_r1,
        }