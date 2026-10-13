import heapq
from dataclasses import dataclass
from typing import FrozenSet, Optional, Tuple

from .plan_builder import PlanBuilder
from ..core.kinematics import KinematicsEngine


DEBUG_STATE_SPACE_EVALUATOR = False

AUTO_THRESHOLD_2_TO_3 = 3.0
AUTO_THRESHOLD_3_TO_4 = 3.5

EPS = 1e-6
MAX_EXPANSIONS = 100000

ENTRANCE_NODES = [1, 2, 3]
EXIT_NODE = 13


@dataclass(frozen=True)
class SearchState:
    pos: int
    heading: int
    outside_front: Optional[int]
    picked: FrozenSet[int]
    pickup_order: Tuple[int, ...]
    plan: Tuple[dict, ...]
    cost: float


class StateSpaceEvaluator:
    """
    State-space evaluator.
    """

    def __init__(
        self,
        search_engine=None,
        max_capacity=4,
        max_expansions=MAX_EXPANSIONS,
    ):
        self.search = search_engine
        self.max_capacity = int(max_capacity)
        self.max_expansions = int(max_expansions)

    def generate(
        self,
        current_node,
        current_heading,
        r2_targets,
        r1_kfs,
        fake_walls,
        dynamic_obstacles,
        team,
        current_payload=0,
        visited_history=None,
        retrieval_mode="auto",
    ):
        visited_history = visited_history or []

        r2_targets = self._clean_nodes(r2_targets)
        r1_kfs = set(self._clean_nodes(r1_kfs))
        fake_walls = self._clean_nodes(fake_walls)
        dynamic_obstacles = self._clean_nodes(dynamic_obstacles)

        r2_targets = [
            target
            for target in r2_targets
            if target not in r1_kfs
        ]

        mode_str = str(retrieval_mode).lower()
        physical_allowed = max(0, self.max_capacity - int(current_payload))

        if physical_allowed <= 0 or not r2_targets:
            plan, cost = self.evaluate(
                current_node=current_node,
                current_heading=current_heading,
                r2_targets=r2_targets,
                r1_kfs=r1_kfs,
                fake_walls=fake_walls,
                dynamic_obstacles=dynamic_obstacles,
                team=team,
                target_pick_count=0,
                visited_history=visited_history,
            )

            return {
                "plan": plan,
                "total_cost": cost,
            }

        def capacity_to_reach_payload(target_payload):
            return min(
                max(0, int(target_payload) - int(current_payload)),
                physical_allowed,
                len(r2_targets),
            )

        if mode_str in ["2", "3", "4"]:
            candidate_capacities = [
                capacity_to_reach_payload(int(mode_str)),
            ]
        else:
            candidate_capacities = [
                capacity_to_reach_payload(4),
                capacity_to_reach_payload(3),
                capacity_to_reach_payload(2),
            ]

        candidate_capacities = [
            capacity
            for capacity in dict.fromkeys(candidate_capacities)
            if capacity >= 0
        ]

        candidates = []

        for capacity in candidate_capacities:
            plan, cost = self.evaluate(
                current_node=current_node,
                current_heading=current_heading,
                r2_targets=r2_targets,
                r1_kfs=r1_kfs,
                fake_walls=fake_walls,
                dynamic_obstacles=dynamic_obstacles,
                team=team,
                target_pick_count=capacity,
                visited_history=visited_history,
            )

            if plan:
                candidates.append((capacity, plan, cost))

        if not candidates:
            return {
                "plan": [],
                "total_cost": float("inf"),
            }

        if mode_str not in ["2", "3", "4"] and len(candidates) > 1:
            _, best_plan, best_cost = self._select_auto_candidate(
                candidates=candidates,
                team=team,
            )

            return {
                "plan": best_plan,
                "total_cost": best_cost,
            }

        _, best_plan, best_cost = min(
            candidates,
            key=lambda item: (
                round(float(item[2]), 6),
                self._front_kfs_score_from_plan(item[1], team),
                self._pickup_order_score_from_plan(item[1], team),
                self._smoothness_score(item[1]),
            ),
        )

        return {
            "plan": best_plan,
            "total_cost": best_cost,
        }

    def evaluate(
        self,
        current_node,
        current_heading,
        r2_targets,
        r1_kfs,
        fake_walls,
        dynamic_obstacles,
        team,
        target_pick_count,
        visited_history=None,
    ):
        visited_history = set(visited_history or [])

        r1_kfs = set(self._clean_nodes(r1_kfs))
        r2_targets = set(self._clean_nodes(r2_targets)) - r1_kfs

        hard_obstacles = (
            set(self._clean_nodes(fake_walls))
            | set(self._clean_nodes(dynamic_obstacles))
        )

        target_pick_count = min(
            int(target_pick_count),
            len(r2_targets),
            self.max_capacity,
        )

        initial_state = SearchState(
            pos=int(current_node),
            heading=KinematicsEngine.normalize_angle(int(current_heading)),
            outside_front=None,
            picked=frozenset(),
            pickup_order=tuple(),
            plan=tuple(),
            cost=0.0,
        )

        queue = []
        counter = 0
        best_seen = {}

        best_goal_state = None
        best_goal_score = None

        self._push_state(
            queue=queue,
            state=initial_state,
            counter=counter,
            team=team,
        )
        counter += 1

        expansions = 0

        while queue:
            priority, _, state = heapq.heappop(queue)

            if best_goal_state is not None:
                if priority[0] > round(best_goal_state.cost, 6) + EPS:
                    break

            state_key = self._state_key(state)
            previous_cost = best_seen.get(state_key)

            if previous_cost is not None and previous_cost <= state.cost + EPS:
                continue

            best_seen[state_key] = state.cost
            expansions += 1

            if expansions > self.max_expansions:
                if DEBUG_STATE_SPACE_EVALUATOR:
                    print("[STATE SPACE] max expansions reached")
                break

            if self._is_goal_state(
                state=state,
                target_pick_count=target_pick_count,
            ):
                goal_score = self._final_goal_score(state, team)

                if best_goal_score is None or goal_score < best_goal_score:
                    best_goal_score = goal_score
                    best_goal_state = state

                continue

            next_states = self._expand_state(
                state=state,
                r2_targets=r2_targets,
                r1_kfs=r1_kfs,
                hard_obstacles=hard_obstacles,
                target_pick_count=target_pick_count,
                team=team,
                visited_history=visited_history,
            )

            for next_state in next_states:
                self._push_state(
                    queue=queue,
                    state=next_state,
                    counter=counter,
                    team=team,
                )
                counter += 1

        if best_goal_state is None:
            return [], float("inf")

        return list(best_goal_state.plan), round(best_goal_state.cost, 3)

    def _expand_state(
        self,
        state,
        r2_targets,
        r1_kfs,
        hard_obstacles,
        target_pick_count,
        team,
        visited_history,
    ):
        if state.pos == EXIT_NODE:
            return []

        if state.pos == 0:
            return self._expand_outside_state(
                state=state,
                r2_targets=r2_targets,
                r1_kfs=r1_kfs,
                hard_obstacles=hard_obstacles,
                target_pick_count=target_pick_count,
                team=team,
            )

        return self._expand_inside_state(
            state=state,
            r2_targets=r2_targets,
            r1_kfs=r1_kfs,
            hard_obstacles=hard_obstacles,
            target_pick_count=target_pick_count,
            team=team,
            visited_history=visited_history,
        )
    
    def _has_picked_entrance_target(self, state):
        return any(
            node in ENTRANCE_NODES
            for node in state.picked
        )

    def _expand_outside_state(
        self,
        state,
        r2_targets,
        r1_kfs,
        hard_obstacles,
        target_pick_count,
        team,
    ):
        next_states = []
        front = state.outside_front

        if self._must_pick_outside_front(
            state=state,
            r2_targets=r2_targets,
            target_pick_count=target_pick_count,
        ):
            return [
                self._apply_action(
                    state=state,
                    target=front,
                    team=team,
                    action_type="PICK_UP",
                    next_pos=0,
                    r1_kfs=r1_kfs,
                    add_pickup_target=front,
                )
            ]

        blocked = self._blocked_nodes(
            r2_targets=r2_targets,
            picked=state.picked,
            hard_obstacles=hard_obstacles,
        )

        locked_front = self._locked_front_after_outside_pick(
            state=state,
            r2_targets=r2_targets,
            target_pick_count=target_pick_count,
        )

        if locked_front is not None:
            if locked_front not in blocked:
                return [
                    self._apply_action(
                        state=state,
                        target=locked_front,
                        team=team,
                        action_type="ENTER",
                        next_pos=locked_front,
                        r1_kfs=r1_kfs,
                    )
                ]

            return []

        unpicked_entrance_targets = self._unpicked_entrance_targets(
            r2_targets=r2_targets,
            picked=state.picked,
            target_pick_count=target_pick_count,
        )

        has_picked_entrance = self._has_picked_entrance_target(state)

        if unpicked_entrance_targets and not has_picked_entrance:
            approach_candidates = unpicked_entrance_targets
            allow_enter_current_front = False
        else:
            approach_candidates = ENTRANCE_NODES
            allow_enter_current_front = True

        for candidate_front in approach_candidates:
            if candidate_front in hard_obstacles:
                continue

            if state.outside_front == candidate_front:
                continue

            next_states.append(
                self._apply_action(
                    state=state,
                    target=candidate_front,
                    team=team,
                    action_type="APPROACH",
                    next_pos=0,
                    r1_kfs=r1_kfs,
                )
            )

        if (
            allow_enter_current_front
            and front in ENTRANCE_NODES
            and front not in blocked
        ):
            next_states.append(
                self._apply_action(
                    state=state,
                    target=front,
                    team=team,
                    action_type="ENTER",
                    next_pos=front,
                    r1_kfs=r1_kfs,
                )
            )

        return next_states

    def _expand_inside_state(
        self,
        state,
        r2_targets,
        r1_kfs,
        hard_obstacles,
        target_pick_count,
        team,
        visited_history,
    ):
        next_states = []

        blocked = self._blocked_nodes(
            r2_targets=r2_targets,
            picked=state.picked,
            hard_obstacles=hard_obstacles,
        )

        pickup_states = []

        if len(state.picked) < target_pick_count:
            for target in sorted(r2_targets):
                if target in state.picked:
                    continue

                if not self._can_pick_from_position(
                    curr_pos=state.pos,
                    target=target,
                    blocked=blocked,
                ):
                    continue

                pickup_states.append(
                    self._apply_action(
                        state=state,
                        target=target,
                        team=team,
                        action_type="PICK_UP",
                        next_pos=state.pos,
                        r1_kfs=r1_kfs,
                        add_pickup_target=target,
                    )
                )

        next_states.extend(pickup_states)

        for neighbor in KinematicsEngine.get_neighbors(state.pos, blocked):
            if neighbor == 0:
                continue

            if neighbor == EXIT_NODE:
                if len(state.picked) < target_pick_count:
                    continue

                next_states.append(
                    self._apply_action(
                        state=state,
                        target=EXIT_NODE,
                        team=team,
                        action_type="EXIT",
                        next_pos=EXIT_NODE,
                        r1_kfs=r1_kfs,
                    )
                )
                continue

            if neighbor in blocked:
                continue

            next_state = self._apply_action(
                state=state,
                target=neighbor,
                team=team,
                action_type="MOVE",
                next_pos=neighbor,
                r1_kfs=r1_kfs,
            )

            if neighbor in visited_history:
                next_state = self._add_visit_penalty_to_state(next_state)

            next_states.append(next_state)

        return next_states

    def _apply_action(
        self,
        state,
        target,
        team,
        action_type,
        next_pos,
        r1_kfs,
        add_pickup_target=None,
    ):
        builder = PlanBuilder.from_existing(
            actions=state.plan,
            total_cost=state.cost,
            outside_front=state.outside_front,
        )

        next_heading = builder.add_action(
            curr_pos=state.pos,
            target=target,
            curr_heading=state.heading,
            team=team,
            action_type=action_type,
            r1_kfs=r1_kfs,
        )

        picked = set(state.picked)
        pickup_order = list(state.pickup_order)

        if add_pickup_target is not None:
            picked.add(add_pickup_target)
            pickup_order.append(add_pickup_target)

        next_plan, next_cost = builder.result()

        return SearchState(
            pos=int(next_pos),
            heading=KinematicsEngine.normalize_angle(int(next_heading)),
            outside_front=builder.outside_front,
            picked=frozenset(picked),
            pickup_order=tuple(pickup_order),
            plan=tuple(next_plan),
            cost=round(next_cost, 6),
        )

    def _add_visit_penalty_to_state(
        self,
        state,
        amount=0.05,
    ):
        plan = [dict(action) for action in state.plan]

        if plan:
            old_cost = float(plan[-1].get("cost", 0.0) or 0.0)
            plan[-1]["cost"] = round(old_cost + amount, 3)

        return SearchState(
            pos=state.pos,
            heading=state.heading,
            outside_front=state.outside_front,
            picked=state.picked,
            pickup_order=state.pickup_order,
            plan=tuple(plan),
            cost=round(state.cost + amount, 6),
        )

    def _blocked_nodes(
        self,
        r2_targets,
        picked,
        hard_obstacles,
    ):
        unpicked_r2 = set(r2_targets) - set(picked)
        return set(hard_obstacles) | unpicked_r2

    def _can_pick_from_position(
        self,
        curr_pos,
        target,
        blocked,
    ):
        if target in [0, EXIT_NODE]:
            return False

        effective_blocked = set(blocked)
        effective_blocked.discard(target)

        valid_stands = KinematicsEngine.get_neighbors(
            target,
            effective_blocked,
        )

        valid_stands = [
            node
            for node in valid_stands
            if node not in [0, EXIT_NODE]
        ]

        return curr_pos in valid_stands

    def _must_pick_outside_front(
        self,
        state,
        r2_targets,
        target_pick_count,
    ):
        front = state.outside_front

        return (
            front in ENTRANCE_NODES
            and front in r2_targets
            and front not in state.picked
            and len(state.picked) < target_pick_count
        )

    def _unpicked_entrance_targets(
        self,
        r2_targets,
        picked,
        target_pick_count,
    ):
        if len(picked) >= target_pick_count:
            return []

        return [
            node
            for node in ENTRANCE_NODES
            if node in r2_targets
            and node not in picked
        ]

    @staticmethod
    def _forward_node(node):
        next_node = int(node) + 3
        return next_node if next_node <= 12 else None

    def _locked_front_after_outside_pick(
        self,
        state,
        r2_targets,
        target_pick_count,
    ):
        front = state.outside_front

        if front not in ENTRANCE_NODES:
            return None

        if front not in state.picked:
            return None

        if len(state.picked) >= target_pick_count:
            return None

        forward = self._forward_node(front)

        if forward in r2_targets and forward not in state.picked:
            return front

        return None

    def _is_goal_state(
        self,
        state,
        target_pick_count,
    ):
        return (
            state.pos == EXIT_NODE
            and len(state.picked) >= target_pick_count
        )

    def _push_state(
        self,
        queue,
        state,
        counter,
        team,
    ):
        priority = (
            round(state.cost, 6),                  
            -len(state.picked),                   
            self._front_kfs_score(state.pickup_order, team),
            self._pickup_order_score(state.pickup_order, team),
            self._smoothness_score(state.plan),
        )

        heapq.heappush(
            queue,
            (
                priority,
                counter,
                state,
            ),
        )

    def _state_key(self, state):
        outside_front = state.outside_front if state.pos == 0 else None

        return (
            state.pos,
            state.heading,
            outside_front,
            tuple(sorted(state.picked)),
            state.pickup_order,
        )

    def _final_goal_score(
        self,
        state,
        team,
    ):
        return (
            round(state.cost, 6),
            self._front_kfs_score(state.pickup_order, team),
            self._pickup_order_score(state.pickup_order, team),
            self._smoothness_score(state.plan),
        )

    def _select_auto_candidate(
        self,
        candidates,
        team,
    ):
        by_capacity = {}

        for capacity, plan, cost in candidates:
            score = (
                round(cost, 6),
                self._front_kfs_score_from_plan(plan, team),
                self._pickup_order_score_from_plan(plan, team),
                self._smoothness_score(plan),
            )

            previous = by_capacity.get(capacity)

            if previous is None:
                by_capacity[capacity] = (plan, cost, score)
                continue

            if score < previous[2]:
                by_capacity[capacity] = (plan, cost, score)

        capacities = sorted(by_capacity.keys())
        best_capacity = capacities[0]

        for capacity in capacities[1:]:
            current_plan, current_cost, _ = by_capacity[best_capacity]
            next_plan, next_cost, _ = by_capacity[capacity]

            threshold = self._auto_upgrade_threshold(
                from_capacity=best_capacity,
                to_capacity=capacity,
            )

            if (
                next_cost - current_cost <= threshold
                or next_cost < current_cost - EPS
            ):
                best_capacity = capacity

        best_plan, best_cost, _ = by_capacity[best_capacity]
        return best_capacity, best_plan, best_cost

    def _auto_upgrade_threshold(
        self,
        from_capacity,
        to_capacity,
    ):
        threshold = 0.0

        for capacity in range(int(from_capacity), int(to_capacity)):
            if capacity == 2:
                threshold += AUTO_THRESHOLD_2_TO_3
            elif capacity == 3:
                threshold += AUTO_THRESHOLD_3_TO_4
            else:
                threshold += AUTO_THRESHOLD_2_TO_3

        return threshold

    def _front_kfs_score_from_plan(
        self,
        plan,
        team,
    ):
        return self._front_kfs_score(
            picked_order=self._picked_order_from_plan(plan),
            team=team,
        )

    def _pickup_order_score_from_plan(
        self,
        plan,
        team,
    ):
        return self._pickup_order_score(
            picked_order=self._picked_order_from_plan(plan),
            team=team,
        )

    def _picked_order_from_plan(
        self,
        plan,
    ):
        picked = []

        for step in plan:
            if self._action_name(step) != "PICK_UP":
                continue

            target = step.get("target")

            if isinstance(target, int):
                picked.append(target)

        return tuple(picked)

    def _front_kfs_score(
        self,
        picked_order,
        team,
    ):
        return sum(
            self._node_front_rank(node, team)
            for node in picked_order
        )

    def _pickup_order_score(
        self,
        picked_order,
        team,
    ):
        return tuple(
            self._node_front_rank(node, team)
            for node in picked_order
        )

    def _node_front_rank(
        self,
        node,
        team,
    ):
        if node < 1 or node > 12:
            return 999

        row = (node - 1) // 3
        col_node = ((node - 1) % 3) + 1
        lane_order = KinematicsEngine.get_entrance_order(team)

        try:
            lane_rank = lane_order.index(col_node)
        except ValueError:
            lane_rank = col_node

        return row * 10 + lane_rank

    def _smoothness_score(
        self,
        plan,
    ):
        if not plan:
            return 0.0

        score = 0.0

        for index, step in enumerate(plan):
            action = self._action_name(step)

            if action == "ROTATE":
                score += 0.10

            if action == "PICK_UP":
                score += index * 0.01

            if action in ["MOVE", "ENTER", "EXIT"]:
                score += index * 0.001

        return score

    @staticmethod
    def _action_name(step):
        return step.get("action_name") or step.get("action")

    @staticmethod
    def _clean_nodes(value):
        result = []

        for item in value or []:
            try:
                node = int(item)
            except Exception:
                continue

            if node not in result:
                result.append(node)

        return result