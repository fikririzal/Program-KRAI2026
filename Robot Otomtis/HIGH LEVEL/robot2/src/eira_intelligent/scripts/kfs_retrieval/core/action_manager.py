from ..config.cost import *
from .kinematics import KinematicsEngine


class ActionManager:
    VALID_ACTIONS = {
        "APPROACH",
        "ENTER",
        "MOVE",
        "PICK_UP",
        "EXIT",
    }

    @staticmethod
    def create_step(
        step_idx,
        node,
        action,
        elev,
        rel_dir,
        heading,
        target,
        cost,
        delta_height=0,
    ):
        return {
            "step": step_idx,
            "node": node,
            "action": action,
            "action_name": action,
            "elev": elev,
            "elevation": elev,
            "delta": delta_height,
            "delta_h": delta_height,
            "delta_height": delta_height,
            "direction": rel_dir,
            "heading": heading,
            "target": target,
            "cost": round(cost, 3),
        }

    @staticmethod
    def _get_elevation_cost(elev_dir, delta_h):
        if elev_dir == "UP":
            return C_ELEV_UP_400 if delta_h == 400 else C_ELEV_UP_200

        if elev_dir == "DOWN":
            return C_ELEV_DOWN_400 if delta_h == 400 else C_ELEV_DOWN_200

        return C_NODE_PENALTY

    @staticmethod
    def _get_pickup_elevation_cost(elev_dir):
        if elev_dir == "UP":
            return C_PICK_ELEV_UP

        if elev_dir == "DOWN":
            return C_PICK_ELEV_DOWN

        return 0.0

    @staticmethod
    def _get_pickup_direction_cost(rel_dir):
        return C_PICK_DIR_FORWARD

    @staticmethod
    def _get_pickup_cost(elev_dir, rel_dir="Forward"):
        return (
            ActionManager._get_pickup_elevation_cost(elev_dir)
            + ActionManager._get_pickup_direction_cost(rel_dir)
        )

    @staticmethod
    def get_validate_cost(elev_dir, target_node=None):
        if elev_dir == "UP":
            elev_cost = C_VALIDATE_UP
        else:
            elev_cost = C_VALIDATE_DOWN

        row_cost = 0.0

        if target_node is not None:
            try:
                node = int(target_node)
            except Exception:
                node = None

            if node is not None and 1 <= node <= 12:
                row = (node - 1) // 3
                row_cost = C_VALIDATE_ROW_PENALTY.get(row, 0.0)

        return elev_cost + row_cost

    @staticmethod
    def _relative_direction_from_angle(angle_diff):
        angle_diff = KinematicsEngine.normalize_angle(angle_diff)

        if angle_diff == 0:
            return "Forward"

        if angle_diff == 90:
            return "Right"

        if angle_diff == -90:
            return "Left"

        if abs(angle_diff) == 180:
            return "Backward"

        if angle_diff > 0:
            return "Right"

        return "Left"

    @staticmethod
    def _add_rotate_if_needed(
        actions,
        step_idx,
        node,
        curr_h,
        angle_diff,
    ):
        angle_diff = KinematicsEngine.normalize_angle(angle_diff)

        if angle_diff == 0:
            return curr_h, step_idx, 0.0

        rot_cost = C_ROT_180 if abs(angle_diff) == 180 else C_ROT_90
        rel_dir = ActionManager._relative_direction_from_angle(angle_diff)
        new_heading = KinematicsEngine.normalize_angle(curr_h + angle_diff)

        actions.append(
            ActionManager.create_step(
                step_idx=step_idx,
                node=node,
                action="ROTATE",
                elev="FLAT",
                rel_dir=rel_dir,
                heading=new_heading,
                target=-1,
                cost=rot_cost,
                delta_height=0,
            )
        )

        return new_heading, step_idx + 1, rot_cost

    @staticmethod
    def _get_elevation_info(node_a, target_b, action_type):
        if action_type == "PICK_UP" and hasattr(
            KinematicsEngine,
            "get_pickup_elevation_info",
        ):
            return KinematicsEngine.get_pickup_elevation_info(
                node_a,
                target_b,
            )

        if action_type in ["MOVE", "ENTER", "EXIT"] and hasattr(
            KinematicsEngine,
            "get_move_elevation_info",
        ):
            return KinematicsEngine.get_move_elevation_info(
                node_a,
                target_b,
            )

        return KinematicsEngine.get_elevation_info(node_a, target_b)

    @staticmethod
    def solve_action(
        node_a,
        target_b,
        curr_h,
        team,
        step_idx,
        action_type="MOVE",
        outside_front=None,
    ):
        action_type = str(action_type).upper()

        if action_type not in ActionManager.VALID_ACTIONS:
            raise ValueError(
                f"Unsupported action_type={action_type}. "
                "VALIDATE dan PUT_DOWN sudah tidak boleh jadi action step. "
                "Validate cost harus digabung ke MOVE/ENTER dari evaluator."
            )

        actions = []
        total_cost = 0.0

        global_dir = KinematicsEngine.get_global_direction(
            node_a,
            target_b,
            team,
        )

        angle_diff = KinematicsEngine.normalize_angle(global_dir - curr_h)
        relative_direction = ActionManager._relative_direction_from_angle(
            angle_diff
        )

        movement_actions = {"MOVE", "ENTER", "EXIT"}

        if action_type in movement_actions:
            curr_h, step_idx, rot_cost = ActionManager._add_rotate_if_needed(
                actions=actions,
                step_idx=step_idx,
                node=node_a,
                curr_h=curr_h,
                angle_diff=angle_diff,
            )
            total_cost += rot_cost
            relative_direction = "Forward"

        elif action_type == "PICK_UP":
            curr_h, step_idx, rot_cost = ActionManager._add_rotate_if_needed(
                actions=actions,
                step_idx=step_idx,
                node=node_a,
                curr_h=curr_h,
                angle_diff=angle_diff,
            )
            total_cost += rot_cost
            relative_direction = "Forward"

        elev_dir, delta_h = ActionManager._get_elevation_info(
            node_a,
            target_b,
            action_type,
        )

        action_cost = 0.0
        rel_dir = relative_direction

        if action_type == "APPROACH":
            elev_dir = "OUTSIDE"

            rel_dir, delta_h = KinematicsEngine.get_approach_shift_info(
                team=team,
                from_front=outside_front,
                to_front=target_b,
            )

            if outside_front is None:
                action_cost = C_APPROACH_INITIAL[target_b]
            else:
                action_cost = delta_h * C_APPROACH_SHIFT_PER_NODE

        elif action_type in ["MOVE", "ENTER"]:
            rel_dir = "Forward"
            action_cost = ActionManager._get_elevation_cost(
                elev_dir,
                delta_h,
            )

        elif action_type == "PICK_UP":
            rel_dir = "Forward"
            action_cost = ActionManager._get_pickup_cost(
                elev_dir=elev_dir,
                rel_dir=rel_dir,
            )

        elif action_type == "EXIT":
            elev_dir = "DOWN"
            rel_dir = "Forward"
            action_cost = ActionManager._get_elevation_cost(
                elev_dir,
                delta_h,
            )

            if node_a == 10:
                action_cost += C_EXIT_NODE_10
            elif node_a == 11:
                action_cost += C_EXIT_NODE_11
            elif node_a == 12:
                action_cost += C_EXIT_NODE_12

        actions.append(
            ActionManager.create_step(
                step_idx=step_idx,
                node=node_a,
                action=action_type,
                elev=elev_dir,
                rel_dir=rel_dir,
                heading=curr_h,
                target=target_b,
                cost=action_cost,
                delta_height=delta_h,
            )
        )

        return actions, total_cost + action_cost, curr_h, step_idx + 1