import heapq

from ..config.cost import *
from ..core.kinematics import KinematicsEngine


class DijkstraSearch:
    name = "Dijkstra Kinematic Planner"

    def __init__(self):
        self.path_cache = {}
        self.candidates_cache = {}

    def _get_elevation_cost(self, elev_dir, delta_h):
        if elev_dir == "UP":
            return C_ELEV_UP_400 if delta_h == 400 else C_ELEV_UP_200

        if elev_dir == "DOWN":
            return C_ELEV_DOWN_400 if delta_h == 400 else C_ELEV_DOWN_200

        return 0.0

    def _get_validate_cost(self, elev_dir, target_node):
        if elev_dir == "UP":
            elev_cost = C_VALIDATE_UP
        else:
            elev_cost = C_VALIDATE_DOWN

        row_cost = 0.0

        try:
            node = int(target_node)
        except Exception:
            node = None

        if node is not None and 1 <= node <= 12:
            row = (node - 1) // 3
            row_cost = C_VALIDATE_ROW_PENALTY.get(row, 0.0)

        return elev_cost + row_cost

    def _get_approach_cost(self, team, outside_front, target):
        if outside_front is None:
            return C_APPROACH_INITIAL[target]

        _, delta = KinematicsEngine.get_approach_shift_info(
            team=team,
            from_front=outside_front,
            to_front=target,
        )

        return delta * C_APPROACH_SHIFT_PER_NODE

    def _get_valid_goals(self, target_node, obstacles, stand_and_reach):
        if target_node in [0, 13]:
            return {target_node}

        if stand_and_reach:
            return {
                node
                for node in KinematicsEngine.get_neighbors(target_node, obstacles)
                if node != 13
            }

        return {target_node}

    def _make_cache_key(
        self,
        start_node,
        start_heading,
        target_node,
        obstacles,
        team,
        r1_kfs,
        visited_history,
        stand_and_reach,
        outside_front,
    ):
        return (
            start_node,
            start_heading,
            target_node,
            tuple(sorted(obstacles)),
            tuple(sorted(r1_kfs)),
            tuple(sorted(visited_history)),
            stand_and_reach,
            team,
            outside_front,
        )

    def _get_step_cost(
        self,
        curr_node,
        curr_heading,
        neighbor,
        target_node,
        team,
        outside_front,
        r1_kfs,
        visited_history,
    ):
        global_dir = KinematicsEngine.get_global_direction(
            curr_node,
            neighbor,
            team,
        )

        angle_diff = KinematicsEngine.normalize_angle(
            global_dir - curr_heading
        )

        if angle_diff == 0:
            rot_cost = 0.0
        elif abs(angle_diff) == 180:
            rot_cost = C_ROT_180
        else:
            rot_cost = C_ROT_90

        elev_dir, delta_h = KinematicsEngine.get_elevation_info(
            curr_node,
            neighbor,
        )

        move_cost = self._get_elevation_cost(
            elev_dir=elev_dir,
            delta_h=delta_h,
        )

        if curr_node == 0 and neighbor in [1, 2, 3]:
            move_cost += self._get_approach_cost(
                team=team,
                outside_front=outside_front,
                target=neighbor,
            )

        if neighbor == 13:
            move_cost += {
                10: C_EXIT_NODE_10,
                11: C_EXIT_NODE_11,
                12: C_EXIT_NODE_12,
            }.get(curr_node, 0.0)

        validate_cost = 0.0

        if neighbor in r1_kfs and neighbor != target_node and neighbor != 13:
            validate_cost = self._get_validate_cost(
                elev_dir=elev_dir,
                target_node=neighbor,
            )

        history_penalty = 0.05 if neighbor in visited_history else 0.0

        total_cost = (
            rot_cost
            + move_cost
            + validate_cost
            + C_NODE_PENALTY
            + history_penalty
        )

        new_heading = KinematicsEngine.normalize_angle(
            curr_heading + angle_diff
        )

        return total_cost, new_heading

    def find_path(
        self,
        start_node,
        start_heading,
        target_node,
        obstacles,
        team,
        r1_kfs=None,
        visited_history=None,
        stand_and_reach=True,
        outside_front=None,
    ):
        candidates = self.find_path_candidates(
            start_node=start_node,
            start_heading=start_heading,
            target_node=target_node,
            obstacles=obstacles,
            team=team,
            r1_kfs=r1_kfs,
            visited_history=visited_history,
            stand_and_reach=stand_and_reach,
            outside_front=outside_front,
            max_candidates=1,
        )

        if not candidates:
            return None, float("inf")

        best = candidates[0]
        return best["path"], best["cost"]

    def find_path_candidates(
        self,
        start_node,
        start_heading,
        target_node,
        obstacles,
        team,
        r1_kfs=None,
        visited_history=None,
        stand_and_reach=True,
        outside_front=None,
        max_candidates=None,
    ):
        r1_kfs = r1_kfs or []
        visited_history = visited_history or []

        if max_candidates is not None and max_candidates <= 0:
            return []

        valid_goals = self._get_valid_goals(
            target_node=target_node,
            obstacles=obstacles,
            stand_and_reach=stand_and_reach,
        )

        if start_node in valid_goals:
            return [
                {
                    "goal": start_node,
                    "path": [start_node],
                    "cost": 0.0,
                    "heading": start_heading,
                }
            ]

        cache_key = self._make_cache_key(
            start_node=start_node,
            start_heading=start_heading,
            target_node=target_node,
            obstacles=obstacles,
            team=team,
            r1_kfs=r1_kfs,
            visited_history=visited_history,
            stand_and_reach=stand_and_reach,
            outside_front=outside_front,
        )

        if cache_key in self.candidates_cache:
            candidates = self.candidates_cache[cache_key]
            if max_candidates is not None:
                return candidates[:max_candidates]

            return candidates

        queue = [
            (
                0.0,
                start_node,
                start_heading,
                [start_node],
            )
        ]

        visited = {}
        found_by_goal = {}

        while queue:
            curr_cost, curr_node, curr_heading, path = heapq.heappop(queue)

            state_key = (
                curr_node,
                curr_heading,
            )

            if state_key in visited and visited[state_key] <= curr_cost:
                continue

            visited[state_key] = curr_cost

            if curr_node in valid_goals and curr_node not in found_by_goal:
                found_by_goal[curr_node] = {
                    "goal": curr_node,
                    "path": path,
                    "cost": curr_cost,
                    "heading": curr_heading,
                }

                if len(found_by_goal) >= len(valid_goals):
                    break

            for neighbor in KinematicsEngine.get_neighbors(curr_node, obstacles):
                step_cost, new_heading = self._get_step_cost(
                    curr_node=curr_node,
                    curr_heading=curr_heading,
                    neighbor=neighbor,
                    target_node=target_node,
                    team=team,
                    outside_front=outside_front,
                    r1_kfs=r1_kfs,
                    visited_history=visited_history,
                )

                new_cost = curr_cost + step_cost

                heapq.heappush(
                    queue,
                    (
                        new_cost,
                        neighbor,
                        new_heading,
                        path + [neighbor],
                    ),
                )

        candidates = list(found_by_goal.values())

        candidates.sort(
            key=lambda item: (
                item["cost"],
                len(item["path"]),
                item["goal"],
            )
        )

        self.candidates_cache[cache_key] = candidates

        if candidates:
            best = candidates[0]
            self.path_cache[cache_key] = (
                best["path"],
                best["cost"],
            )

        if max_candidates is not None:
            return candidates[:max_candidates]

        return candidates