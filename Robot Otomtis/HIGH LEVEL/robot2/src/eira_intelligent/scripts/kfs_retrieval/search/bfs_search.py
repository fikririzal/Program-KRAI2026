from collections import deque

from ..core.kinematics import KinematicsEngine


class BFSSearch:
    name = "BFS Planner"

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

        queue = deque([(start_node, [start_node])])
        visited = {start_node}
        found_by_goal = {}

        while queue:
            current, path = queue.popleft()

            for neighbor in KinematicsEngine.get_neighbors(current, obstacles):
                if neighbor in visited:
                    continue

                new_path = path + [neighbor]
                visited.add(neighbor)

                if neighbor in valid_goals and neighbor not in found_by_goal:
                    found_by_goal[neighbor] = {
                        "goal": neighbor,
                        "path": new_path,
                        "cost": float(len(new_path) - 1),
                        "heading": start_heading,
                    }

                    if len(found_by_goal) >= len(valid_goals):
                        break

                queue.append((neighbor, new_path))

            if len(found_by_goal) >= len(valid_goals):
                break

        candidates = list(found_by_goal.values())
        candidates.sort(
            key=lambda item: (
                item["cost"],
                len(item["path"]),
                item["goal"],
            )
        )

        return candidates[:max_candidates] if max_candidates is not None else candidates