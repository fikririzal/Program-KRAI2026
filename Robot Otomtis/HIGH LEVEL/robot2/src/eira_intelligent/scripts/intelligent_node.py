from typing import Any, List, Optional, Sequence, Tuple

import rclpy
from rclpy.node import Node

try:
    from eira_msgs.msg import GuiData as GuiPlanningMsg
except ImportError:
    from eira_msgs.msg import DataGui as GuiPlanningMsg

try:
    from eira_msgs.msg import KfsRetrievalPlan, ForestPlanStep, ForestPathPoint
except ImportError as import_error:
    KfsRetrievalPlan = None
    ForestPlanStep = None
    ForestPathPoint = None
    _MSG_IMPORT_ERROR = import_error
else:
    _MSG_IMPORT_ERROR = None

from eira_intelligent.kfs_retrieval.kfs_retrieval import KFSRetrieval
from eira_intelligent.kfs_retrieval.core.kinematics import KinematicsEngine


class IntelligentNode(Node):
    TEAM_MAP = {
        0: "blue",
        1: "red",
        "0": "blue",
        "1": "red",
        "blue": "blue",
        "red": "red",
    }

    VALID_MIN_NODE = 1
    VALID_MAX_NODE = 12

    def __init__(self):
        super().__init__("intelligent_node")

        if (
            KfsRetrievalPlan is None
            or ForestPlanStep is None
            or ForestPathPoint is None
        ):
            raise RuntimeError(
                "eira_msgs/msg/KfsRetrievalPlan, ForestPlanStep, atau "
                "ForestPathPoint belum tersedia. "
                "Update msg, rebuild eira_msgs, lalu rebuild eira_intelligent. "
                f"Import error: {_MSG_IMPORT_ERROR}"
            )

        self._declare_parameters()
        self._load_parameters()

        self.planner = KFSRetrieval()

        self.plan_pub = self.create_publisher(
            KfsRetrievalPlan,
            self.output_topic,
            1,
        )

        self.gui_sub = self.create_subscription(
            GuiPlanningMsg,
            self.input_topic,
            self._gui_callback,
            10,
        )

        self._last_signature: Optional[Tuple[Any, ...]] = None
        self._latest_plan_msg: Optional[KfsRetrievalPlan] = None

        self.plan_publish_timer = self.create_timer(
            1.0 / self.publish_rate_hz,
            self._publish_latest_plan,
        )

        self.get_logger().info(
            "[IntelligentNode] KFS planner ready | "
            f"input={self.input_topic}, output={self.output_topic}, "
            f"publish_rate={self.publish_rate_hz:.2f} Hz, "
            "generate=team/kfsret/forest change only, "
            "kfsret: 0=2KFS, 1=3KFS, 2=4KFS, 3=auto"
        )

    def _declare_parameters(self) -> None:
        self.declare_parameter("input_topic", "/gui_data")
        self.declare_parameter("output_topic", "/intelligent/kfs_retrieval_plan")
        self.declare_parameter("r2_start_node", 0)
        self.declare_parameter("r2_start_heading", 0)
        self.declare_parameter("r1_start_node", 0)
        self.declare_parameter("initial_inventory", [])
        self.declare_parameter("print_plan_detail", False)
        self.declare_parameter("publish_rate_hz", 10.0)

    def _load_parameters(self) -> None:
        self.input_topic = str(self.get_parameter("input_topic").value)
        self.output_topic = str(self.get_parameter("output_topic").value)
        self.r2_start_node = int(self.get_parameter("r2_start_node").value)
        self.r2_start_heading = int(self.get_parameter("r2_start_heading").value)
        self.r1_start_node = int(self.get_parameter("r1_start_node").value)

        self.initial_inventory = self._to_int_list(
            self.get_parameter("initial_inventory").value
        )

        self.print_plan_detail = bool(
            self.get_parameter("print_plan_detail").value
        )

        self.publish_rate_hz = float(
            self.get_parameter("publish_rate_hz").value
        )

        if self.publish_rate_hz <= 0.0:
            self.get_logger().warn(
                "[IntelligentNode] publish_rate_hz <= 0. Fallback to 10 Hz."
            )
            self.publish_rate_hz = 10.0

    def _gui_callback(self, msg: GuiPlanningMsg) -> None:
        signature = self._make_signature(msg)

        if signature == self._last_signature:
            return

        self._last_signature = signature
        self._latest_plan_msg = self._generate_plan_msg(msg)
        self._publish_latest_plan()

    def _publish_latest_plan(self) -> None:
        if self._latest_plan_msg is None:
            return

        self.plan_pub.publish(self._latest_plan_msg)

    def _make_signature(self, msg: GuiPlanningMsg) -> Tuple[Any, ...]:
        forest = self._normalize_forest(
            getattr(msg, "forest", []),
            warn=False,
        )

        return (
            int(getattr(msg, "team", 0)),
            int(getattr(msg, "kfsret", 0)),
            tuple(forest),
            int(self.r2_start_node),
            int(self.r2_start_heading),
            int(self.r1_start_node),
            tuple(self.initial_inventory),
        )

    def _generate_plan_msg(self, msg: GuiPlanningMsg) -> KfsRetrievalPlan:
        try:
            team_value = int(getattr(msg, "team", 0))
            kfsret_value = int(getattr(msg, "kfsret", 0))

            team = self._resolve_team(team_value)
            retrieval_mode = self._resolve_retrieval_mode(kfsret_value)
            forest = self._normalize_forest(getattr(msg, "forest", []), warn=True)

            r2_targets, r1_targets, fake_walls = self._parse_forest(forest)

            result = self.planner.plan_best_route(
                team=team,
                r2_pos=self.r2_start_node,
                r2_heading=self.r2_start_heading,
                r2_targets=r2_targets,
                r1_targets=r1_targets,
                fake_walls=fake_walls,
                dynamic_obstacles=[],
                r1_pos=self.r1_start_node,
                inventory=list(self.initial_inventory),
                validated_empty=[],
                visited_history=[],
                retrieval_mode=retrieval_mode,
                verbose=False,
            )

            plan = list(result.get("r2_plan", []) or [])
            path_nodes, picked_kfs = self._extract_result_arrays(plan)
            visual_path = self._extract_visual_path(plan)

            out = self._make_result_msg(
                plan=plan,
                path_nodes=path_nodes,
                picked_kfs=picked_kfs,
                visual_path=visual_path,
                team=team,
                total_cost=float(result.get("r2_total_cost", 0.0) or 0.0),
            )

            if self.print_plan_detail:
                self._print_plan_result(
                    result=result,
                    plan=plan,
                    path_nodes=path_nodes,
                    picked_kfs=picked_kfs,
                    team=team,
                    team_value=team_value,
                    kfsret_value=kfsret_value,
                    retrieval_mode=retrieval_mode,
                    forest=forest,
                    r2_targets=r2_targets,
                    r1_targets=r1_targets,
                    fake_walls=fake_walls,
                )

            return out

        except Exception as error:
            self.get_logger().error(
                f"[IntelligentNode] KFS planning failed | "
                f"{type(error).__name__}: {error}"
            )

            return self._make_result_msg(
                plan=[],
                path_nodes=[],
                picked_kfs=[],
                visual_path=[],
                team="blue",
                total_cost=0.0,
            )

    def _resolve_team(self, value: Any) -> str:
        key = str(value).lower().strip() if not isinstance(value, int) else value

        if key not in self.TEAM_MAP:
            self.get_logger().warn(
                f"[IntelligentNode] Unknown team value={value}. Fallback to blue."
            )
            return "blue"

        return self.TEAM_MAP[key]

    def _resolve_retrieval_mode(self, value: Any) -> str:
        try:
            mode = int(value)
        except Exception:
            self.get_logger().warn(
                f"[IntelligentNode] Invalid kfsret={value}. Fallback to auto."
            )
            return "auto"

        if mode == 0:
            return "2"

        if mode == 1:
            return "3"

        if mode == 2:
            return "4"

        if mode == 3:
            return "auto"

        self.get_logger().warn(
            f"[IntelligentNode] Unsupported kfsret={mode}. Fallback to auto."
        )
        return "auto"

    def _normalize_forest(self, value: Any, warn: bool = True) -> List[int]:
        forest = self._to_int_list(value)

        if len(forest) < 12:
            if warn:
                self.get_logger().warn(
                    f"[IntelligentNode] forest length={len(forest)} < 12. "
                    "Padding with 0."
                )
            forest = forest + [0] * (12 - len(forest))

        if len(forest) > 12:
            if warn:
                self.get_logger().warn(
                    f"[IntelligentNode] forest length={len(forest)} > 12. "
                    "Using first 12."
                )
            forest = forest[:12]

        return forest

    def _parse_forest(
        self,
        forest: Sequence[int],
    ) -> Tuple[List[int], List[int], List[int]]:
        r2_targets: List[int] = []
        r1_targets: List[int] = []
        fake_walls: List[int] = []

        for index, raw_value in enumerate(forest):
            node = index + 1
            value = int(raw_value)

            if value == 1:
                r1_targets.append(node)
            elif value == 2:
                r2_targets.append(node)
            elif value == 3:
                fake_walls.append(node)
            elif value != 0:
                self.get_logger().warn(
                    f"[IntelligentNode] Unknown forest value={value} "
                    f"at node={node}. Ignored."
                )

        return r2_targets, r1_targets, fake_walls

    def _make_result_msg(
        self,
        plan: Sequence[dict],
        path_nodes: Sequence[int],
        picked_kfs: Sequence[int],
        visual_path: Sequence[Tuple[int, int, int]],
        team: str,
        total_cost: float = 0.0,
    ) -> KfsRetrievalPlan:
        out = KfsRetrievalPlan()
        out.stamp = self.get_clock().now().to_msg()
        out.step_count = len(plan)
        out.total_cost = float(total_cost)
        out.path_nodes = [int(item) for item in path_nodes]
        out.picked_kfs = [int(item) for item in picked_kfs]

        out.visual_path = [
            self._make_path_point_msg(zone, node, anchor_node)
            for zone, node, anchor_node in visual_path
        ]

        out.steps = [
            self._make_step_msg(
                step_data=step_data,
                step_index=index,
                team=team,
            )
            for index, step_data in enumerate(plan, start=1)
        ]

        return out

    def _make_path_point_msg(
        self,
        zone: int,
        node: int,
        anchor_node: int,
    ) -> ForestPathPoint:
        msg = ForestPathPoint()
        msg.zone = int(zone)
        msg.node = int(node)
        msg.anchor_node = int(anchor_node)
        return msg

    def _make_step_msg(
        self,
        step_data: dict,
        step_index: int,
        team: str,
    ) -> ForestPlanStep:
        action_name = str(step_data.get("action", "") or "UNKNOWN").upper()
        node = self._safe_int(step_data.get("node"))
        target = self._safe_int(step_data.get("target"))

        msg = ForestPlanStep()
        msg.step = int(step_index)
        msg.node = self._encode_node(node)
        msg.target = self._encode_target(action_name, target)
        msg.action = self._action_to_id(action_name)
        msg.action_name = action_name

        elev, delta, direction = self._derive_step_info(
            step_data=step_data,
            action_name=action_name,
            node=msg.node,
            target=msg.target,
            team=team,
        )

        msg.elev = elev
        msg.delta = int(delta)
        msg.direction = direction

        heading = self._safe_int(step_data.get("heading"))
        msg.heading = int(heading) if heading is not None else 0

        msg.wall = self._get_step_wall_array(
            action_name=action_name,
            node=msg.node,
            heading=msg.heading,
            team=team,
        )

        msg.cost = self._safe_float(step_data.get("cost"), default=0.0)

        return msg

    def _get_step_wall_array(
        self,
        action_name: str,
        node: int,
        heading: int,
        team: str,
    ) -> List[int]:
        action = str(action_name or "").upper()

        if action == "APPROACH":
            return [0, 0, 0, 0]

        if not self._is_valid_arena_node(node):
            return [0, 0, 0, 0]

        try:
            wall = KinematicsEngine.get_relative_wall_array(
                node=node,
                heading=heading,
                team=team,
            )
            return self._normalize_wall_array(wall)
        except Exception as error:
            self.get_logger().warn(
                "[IntelligentNode] Failed to compute wall array | "
                f"node={node}, heading={heading}, team={team}, "
                f"{type(error).__name__}: {error}"
            )
            return [1, 1, 1, 1]

    @staticmethod
    def _normalize_wall_array(value: Any) -> List[int]:
        try:
            items = list(value)
        except Exception:
            return [1, 1, 1, 1]

        result = []

        for item in items[:4]:
            try:
                result.append(1 if int(item) else 0)
            except Exception:
                result.append(1)

        while len(result) < 4:
            result.append(1)

        return result

    def _action_to_id(self, action_name: str) -> int:
        action = str(action_name or "").upper()

        if action == "APPROACH":
            return int(ForestPlanStep.ACTION_APPROACH)
        if action == "ENTER":
            return int(ForestPlanStep.ACTION_ENTER)
        if action == "MOVE":
            return int(ForestPlanStep.ACTION_MOVE)
        if action == "ROTATE":
            return int(ForestPlanStep.ACTION_ROTATE)
        if action == "PICK_UP":
            return int(ForestPlanStep.ACTION_PICK_UP)
        if action == "VALIDATE":
            return int(ForestPlanStep.ACTION_VALIDATE)
        if action == "EXIT":
            return int(ForestPlanStep.ACTION_EXIT)

        return int(ForestPlanStep.ACTION_UNKNOWN)

    def _derive_step_info(
        self,
        step_data: dict,
        action_name: str,
        node: int,
        target: int,
        team: str,
    ) -> Tuple[str, int, str]:
        action = str(action_name or "").upper()

        if action == "ROTATE":
            return "-", 0, "TURN"

        elev = str(
            step_data.get("elev")
            or step_data.get("elevation")
            or step_data.get("elev_dir")
            or ""
        ).strip()

        direction = str(
            step_data.get("direction")
            or step_data.get("dir")
            or step_data.get("move_dir")
            or step_data.get("pickup_dir")
            or ""
        ).strip()

        delta = self._safe_int(
            step_data.get("delta")
            or step_data.get("delta_h")
            or step_data.get("height_delta")
        )

        if delta is None:
            delta = 0

        if self._is_valid_arena_node(node) and self._is_valid_arena_node(target):
            kin_elev, kin_delta = self._get_elevation(node, target)
            kin_direction = self._get_direction(node, target, team)

            if not elev:
                elev = kin_elev

            if delta == 0:
                delta = kin_delta

            if not direction:
                direction = kin_direction

        if not elev:
            elev = "FLAT" if action in ["MOVE", "PICK_UP", "VALIDATE"] else "-"

        if not direction:
            direction = "FORWARD" if action in ["APPROACH", "ENTER", "EXIT"] else "-"

        return elev, int(delta), direction

    def _get_elevation(self, node: int, target: int) -> Tuple[str, int]:
        try:
            if hasattr(KinematicsEngine, "get_elevation_info"):
                info = KinematicsEngine.get_elevation_info(node, target)

                if isinstance(info, tuple) and len(info) >= 2:
                    elev = str(info[0]) if info[0] else "FLAT"
                    delta = self._safe_int(info[1])
                    return elev, int(delta or 0)

            if hasattr(KinematicsEngine, "get_elevation_delta"):
                delta = int(KinematicsEngine.get_elevation_delta(node, target))

                if delta > 0:
                    return "UP", abs(delta)
                if delta < 0:
                    return "DOWN", abs(delta)

        except Exception:
            pass

        return "FLAT", 0

    def _get_direction(self, node: int, target: int, team: str) -> str:
        for method_name in ["get_global_direction", "get_direction"]:
            try:
                if not hasattr(KinematicsEngine, method_name):
                    continue

                method = getattr(KinematicsEngine, method_name)

                try:
                    value = method(node, target, team)
                except TypeError:
                    value = method(node, target)

                if value is not None:
                    return str(value)

            except Exception:
                continue

        return "-"

    def _extract_result_arrays(
        self,
        plan: Sequence[dict],
    ) -> Tuple[List[int], List[int]]:
        path_nodes: List[int] = []
        picked_kfs: List[int] = []

        for step in plan:
            action = str(step.get("action", "") or "").upper()
            node = self._safe_int(step.get("node"))
            target = self._safe_int(step.get("target"))

            if action in ["APPROACH", "ENTER", "MOVE"]:
                if self._is_valid_arena_node(target):
                    self._append_unique_consecutive(path_nodes, int(target))

            elif action == "EXIT":
                if self._is_valid_arena_node(node):
                    self._append_unique_consecutive(path_nodes, int(node))

            if action == "PICK_UP":
                if self._is_valid_arena_node(target):
                    self._append_unique_value(picked_kfs, int(target))

        return path_nodes, picked_kfs

    def _extract_visual_path(
        self,
        plan: Sequence[dict],
    ) -> List[Tuple[int, int, int]]:
        visual_path: List[Tuple[int, int, int]] = []

        for step in plan:
            action = str(step.get("action", "") or "").upper()
            node = self._safe_int(step.get("node"))
            target = self._safe_int(step.get("target"))

            if action == "APPROACH":
                if self._is_valid_arena_node(target):
                    self._append_visual_path_point(
                        visual_path,
                        int(ForestPathPoint.ZONE_OUTSIDE_ENTRY),
                        0,
                        int(target),
                    )

            elif action in ["ENTER", "MOVE"]:
                if self._is_valid_arena_node(target):
                    self._append_visual_path_point(
                        visual_path,
                        int(ForestPathPoint.ZONE_FOREST),
                        int(target),
                        int(target),
                    )

            elif action == "EXIT":
                if self._is_valid_arena_node(node):
                    self._append_visual_path_point(
                        visual_path,
                        int(ForestPathPoint.ZONE_FOREST),
                        int(node),
                        int(node),
                    )
                    self._append_visual_path_point(
                        visual_path,
                        int(ForestPathPoint.ZONE_OUTSIDE_EXIT),
                        13,
                        int(node),
                    )

        return visual_path

    def _append_visual_path_point(
        self,
        points: List[Tuple[int, int, int]],
        zone: int,
        node: int,
        anchor_node: int,
    ) -> None:
        point = (int(zone), int(node), int(anchor_node))

        if not points or points[-1] != point:
            points.append(point)

    def _print_plan_result(
        self,
        result: dict,
        plan: Sequence[dict],
        path_nodes: Sequence[int],
        picked_kfs: Sequence[int],
        team: str,
        team_value: int,
        kfsret_value: int,
        retrieval_mode: str,
        forest: Sequence[int],
        r2_targets: Sequence[int],
        r1_targets: Sequence[int],
        fake_walls: Sequence[int],
    ) -> None:
        self.get_logger().info(
            "\n".join(
                [
                    "",
                    "========== KFS RETRIEVAL PLAN ==========",
                    f"Planner        : {result.get('planner', '-')}",
                    f"Total cost     : {float(result.get('r2_total_cost', 0.0) or 0.0):.2f}",
                    f"Step count     : {len(plan)}",
                    f"Team           : {team} ({team_value})",
                    f"KFS ret        : {kfsret_value}",
                    f"Retrieval mode : {retrieval_mode}",
                    f"Forest         : {list(forest)}",
                    f"R2 targets     : {list(r2_targets)}",
                    f"R1 targets     : {list(r1_targets)}",
                    f"Fake walls     : {list(fake_walls)}",
                    f"Path nodes     : {list(path_nodes)}",
                    f"Picked KFS     : {list(picked_kfs)}",
                    "========================================",
                ]
            )
        )

    @staticmethod
    def _to_int_list(value: Any) -> List[int]:
        if value is None:
            return []

        if isinstance(value, str):
            text = value.strip().replace("[", "").replace("]", "")
            if not text:
                return []

            result = []
            for part in text.split(","):
                try:
                    result.append(int(part.strip()))
                except Exception:
                    pass
            return result

        try:
            return [int(item) for item in list(value)]
        except Exception:
            return []

    @staticmethod
    def _safe_int(value: Any) -> Optional[int]:
        try:
            return int(value)
        except Exception:
            return None

    @staticmethod
    def _safe_float(value: Any, default: float = 0.0) -> float:
        try:
            return float(value)
        except Exception:
            return float(default)

    @classmethod
    def _is_valid_arena_node(cls, value: Optional[int]) -> bool:
        if value is None:
            return False

        return cls.VALID_MIN_NODE <= int(value) <= cls.VALID_MAX_NODE

    @staticmethod
    def _append_unique_consecutive(items: List[int], value: int) -> None:
        if not items or items[-1] != value:
            items.append(value)

    @staticmethod
    def _append_unique_value(items: List[int], value: int) -> None:
        if value not in items:
            items.append(value)

    @staticmethod
    def _encode_node(value: Optional[int]) -> int:
        if value is None:
            return int(ForestPlanStep.NO_NODE)

        return int(value)

    @staticmethod
    def _encode_target(action_name: str, value: Optional[int]) -> int:
        if str(action_name or "").upper() == "ROTATE":
            return int(ForestPlanStep.NO_NODE)

        if value is None:
            return int(ForestPlanStep.NO_NODE)

        return int(value)


def main(args=None):
    rclpy.init(args=args)

    node = IntelligentNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()