from copy import deepcopy

from ..core.action_manager import ActionManager
from ..core.kinematics import KinematicsEngine


class PlanBuilder:
    """
    Helper to build executable action steps.
    """

    def __init__(self, step_idx=1, outside_front=None):
        self.actions = []
        self.total_cost = 0.0
        self.step_idx = step_idx
        self.outside_front = outside_front

    @classmethod
    def from_existing(
        cls,
        actions,
        total_cost,
        outside_front=None,
        step_idx=None,
    ):
        actions = list(actions or [])

        if step_idx is None:
            last_step = 0

            for action in actions:
                try:
                    last_step = max(last_step, int(action.get("step", 0)))
                except Exception:
                    pass

            step_idx = last_step + 1

        builder = cls(
            step_idx=step_idx,
            outside_front=outside_front,
        )
        builder.actions = deepcopy(actions)
        builder.total_cost = float(total_cost or 0.0)

        return builder

    def clone(self):
        cloned = PlanBuilder(
            step_idx=self.step_idx,
            outside_front=self.outside_front,
        )
        cloned.actions = deepcopy(self.actions)
        cloned.total_cost = self.total_cost

        return cloned

    def add_action(
        self,
        curr_pos,
        target,
        curr_heading,
        team,
        action_type,
        r1_kfs=None,
    ):
        r1_kfs = set(r1_kfs or [])
        action_type = str(action_type).upper()

        actions, cost, new_heading, new_step_idx = ActionManager.solve_action(
            node_a=curr_pos,
            target_b=target,
            curr_h=curr_heading,
            team=team,
            step_idx=self.step_idx,
            action_type=action_type,
            outside_front=self.outside_front,
        )

        actions = [dict(action) for action in actions]

        if action_type in ["MOVE", "ENTER"] and target in r1_kfs:
            validate_cost = self._get_validate_cost(
                from_node=curr_pos,
                target=target,
            )

            self._add_validate_cost_to_steps(
                actions=actions,
                target=target,
                validate_cost=validate_cost,
            )

            cost += validate_cost

        self.actions.extend(actions)
        self.total_cost = round(self.total_cost + cost, 6)
        self.step_idx = new_step_idx

        self._update_outside_front(
            curr_pos=curr_pos,
            target=target,
            action_type=action_type,
        )

        return new_heading

    def _update_outside_front(
        self,
        curr_pos,
        target,
        action_type,
    ):
        if curr_pos == 0 and target in [1, 2, 3] and action_type == "APPROACH":
            self.outside_front = target
            return

        if action_type == "ENTER":
            self.outside_front = None
            return

        if curr_pos != 0:
            self.outside_front = None

    def _get_validate_cost(
        self,
        from_node,
        target,
    ):
        if hasattr(KinematicsEngine, "get_move_elevation_info"):
            elev_dir, _ = KinematicsEngine.get_move_elevation_info(
                from_node,
                target,
            )
        else:
            elev_dir, _ = KinematicsEngine.get_elevation_info(
                from_node,
                target,
            )

        return ActionManager.get_validate_cost(
            elev_dir=elev_dir,
            target_node=target,
        )

    def _add_validate_cost_to_steps(
        self,
        actions,
        target,
        validate_cost,
    ):
        for index in range(len(actions) - 1, -1, -1):
            action = actions[index]
            action_name = self._action_name(action)

            if action_name not in ["MOVE", "ENTER"]:
                continue

            if action.get("target") != target:
                continue

            old_cost = float(action.get("cost", 0.0) or 0.0)
            action["cost"] = round(old_cost + validate_cost, 3)
            action["validate_cost"] = round(validate_cost, 3)
            return

    def add_visit_penalty(self, amount=0.05):
        if self.actions:
            old_cost = float(self.actions[-1].get("cost", 0.0) or 0.0)
            self.actions[-1]["cost"] = round(old_cost + amount, 3)

        self.total_cost = round(self.total_cost + amount, 6)

    def extend_path(
        self,
        path,
        curr_pos,
        curr_heading,
        team,
        r1_kfs,
        visited_history=None,
        exit_node=13,
    ):
        visited_history = visited_history or []
        r1_kfs = set(r1_kfs or [])

        for idx in range(len(path) - 1):
            next_node = path[idx + 1]

            if curr_pos == 0 and next_node in [1, 2, 3]:
                if self.outside_front != next_node:
                    curr_heading = self.add_action(
                        curr_pos=curr_pos,
                        target=next_node,
                        curr_heading=curr_heading,
                        team=team,
                        action_type="APPROACH",
                        r1_kfs=r1_kfs,
                    )

                curr_heading = self.add_action(
                    curr_pos=curr_pos,
                    target=next_node,
                    curr_heading=curr_heading,
                    team=team,
                    action_type="ENTER",
                    r1_kfs=r1_kfs,
                )

                if next_node in visited_history:
                    self.add_visit_penalty()

                curr_pos = next_node
                continue

            action_type = "EXIT" if next_node == exit_node else "MOVE"

            curr_heading = self.add_action(
                curr_pos=curr_pos,
                target=next_node,
                curr_heading=curr_heading,
                team=team,
                action_type=action_type,
                r1_kfs=r1_kfs,
            )

            if action_type in ["MOVE", "ENTER"] and next_node in visited_history:
                self.add_visit_penalty()

            curr_pos = next_node

        return curr_pos, curr_heading

    def result(self):
        self._renumber_steps()
        self.total_cost = self._sum_action_costs()

        return self.actions, round(self.total_cost, 6)

    def _renumber_steps(self):
        for index, action in enumerate(self.actions, start=1):
            action["step"] = index

        self.step_idx = len(self.actions) + 1

    def _sum_action_costs(self):
        total = 0.0

        for action in self.actions:
            total += float(action.get("cost", 0.0) or 0.0)

        return round(total, 6)

    @staticmethod
    def _action_name(action):
        action_name = action.get("action_name")

        if action_name:
            return action_name

        action_value = action.get("action")

        if isinstance(action_value, str):
            return action_value

        action_id_map = {
            1: "APPROACH",
            2: "ENTER",
            3: "MOVE",
            4: "ROTATE",
            5: "PICK_UP",
            7: "EXIT",
        }

        return action_id_map.get(action_value, "UNKNOWN")