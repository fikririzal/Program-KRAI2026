from .planning.r1_pathway_planner import R1PathwayPlanner
from .planning.r2_planner import R2Planner
from .search.bfs_search import BFSSearch
from .search.dijkstra_search import DijkstraSearch
from .utils.printer import print_strategy_log


class KFSRetrieval:
    """
    Public API for Behavior Tree.

    Behavior Tree should call this class only, not BFS/Dijkstra directly.
    """

    def __init__(self):
        self.bfs_planner = R2Planner(search_engine=BFSSearch())
        self.dijkstra_planner = R2Planner(search_engine=DijkstraSearch())
        self.r1_planner = R1PathwayPlanner()

    def plan_best_route(
        self,
        team,
        r2_pos,
        r2_heading,
        r2_targets,
        r1_targets,
        fake_walls,
        dynamic_obstacles=None,
        r1_pos=0,
        inventory=None,
        validated_empty=None,
        visited_history=None,
        retrieval_mode="auto",
        verbose=False,
    ):
        dynamic_obstacles = dynamic_obstacles or []
        inventory = inventory or []
        validated_empty = validated_empty or []
        visited_history = visited_history or []

        bfs_strategy = self.bfs_planner.plan(
            team=team,
            r2_pos=r2_pos,
            r2_heading=r2_heading,
            r2_targets=r2_targets,
            r1_targets=r1_targets,
            fake_walls=fake_walls,
            dynamic_obstacles=dynamic_obstacles,
            inventory=inventory,
            validated_empty=validated_empty,
            visited_history=visited_history,
            retrieval_mode=retrieval_mode,
        )

        dijkstra_strategy = self.dijkstra_planner.plan(
            team=team,
            r2_pos=r2_pos,
            r2_heading=r2_heading,
            r2_targets=r2_targets,
            r1_targets=r1_targets,
            fake_walls=fake_walls,
            dynamic_obstacles=dynamic_obstacles,
            inventory=inventory,
            validated_empty=validated_empty,
            visited_history=visited_history,
            retrieval_mode=retrieval_mode,
        )

        best_strategy = self._select_best_strategy(bfs_strategy, dijkstra_strategy)

        best_strategy["r1_rec"] = self.r1_planner.get_recommendation(
            r1_targets=best_strategy.get("clean_r1_targets", []),
            r2_plan=best_strategy["r2_plan"],
            current_r1_pos=r1_pos,
        )

        if verbose:
            print_strategy_log(best_strategy, best_strategy["planner"])

        return best_strategy

    def _select_best_strategy(self, bfs_strategy, dijkstra_strategy):
        bfs_cost = bfs_strategy["r2_total_cost"]
        dijkstra_cost = dijkstra_strategy["r2_total_cost"]

        bfs_valid = bool(bfs_strategy["r2_plan"])
        dijkstra_valid = bool(dijkstra_strategy["r2_plan"])

        if bfs_valid and not dijkstra_valid:
            return bfs_strategy

        if dijkstra_valid and not bfs_valid:
            return dijkstra_strategy

        if not bfs_valid and not dijkstra_valid:
            return {
                "planner": "NONE",
                "r2_plan": [],
                "r2_total_cost": float("inf"),
                "r1_rec": [],
                "clean_r1_targets": [],
            }

        if bfs_cost < dijkstra_cost:
            return bfs_strategy

        # Tie-breaker and smoother movement preference
        return dijkstra_strategy

    def print_plan(self, strategy):
        print_strategy_log(strategy, strategy.get("planner", "Planner"))