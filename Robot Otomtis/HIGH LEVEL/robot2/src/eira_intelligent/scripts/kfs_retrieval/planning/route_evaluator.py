import itertools

from .plan_builder import PlanBuilder
from ..config.cost import THRESHOLD_EXTRA_PICKUP_FOR_EXPENSIVE_EXIT
from ..core.kinematics import KinematicsEngine


DEBUG_ROUTE_EVALUATOR = False

AUTO_THRESHOLD_2_TO_3 = 3.0
AUTO_THRESHOLD_3_TO_4 = 3.5


class RouteEvaluator:
    def __init__(self, search_engine, max_capacity=4):
        self.search = search_engine
        self.max_capacity = max_capacity

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
        r1_kfs = set(r1_kfs or [])

        r2_targets = [
            target
            for target in r2_targets
            if target not in r1_kfs
        ]

        mode_str = str(retrieval_mode).lower()
        physical_allowed = max(0, self.max_capacity - current_payload)

        if physical_allowed <= 0 or not r2_targets:
            plan, cost = self.evaluate(
                current_node=current_node,
                current_heading=current_heading,
                targets=r2_targets,
                r1_kfs=r1_kfs,
                fake_walls=fake_walls,
                dynamic_obstacles=dynamic_obstacles,
                team=team,
                capacity=0,
                current_payload=current_payload,
                visited_history=visited_history,
                allow_extra_pickup=False,
            )

            return {
                "plan": plan,
                "total_cost": cost,
            }

        def capacity_to_reach_payload(target_payload):
            return min(
                max(0, int(target_payload) - current_payload),
                physical_allowed,
                len(r2_targets),
            )

        if mode_str in ["2", "3", "4"]:
            candidate_capacities = [
                capacity_to_reach_payload(int(mode_str)),
            ]
            allow_extra_pickup = False
        else:
            candidate_capacities = [
                capacity_to_reach_payload(4),
                capacity_to_reach_payload(3),
                capacity_to_reach_payload(2),
            ]
            allow_extra_pickup = True

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
                targets=r2_targets,
                r1_kfs=r1_kfs,
                fake_walls=fake_walls,
                dynamic_obstacles=dynamic_obstacles,
                team=team,
                capacity=capacity,
                current_payload=current_payload,
                visited_history=visited_history,
                allow_extra_pickup=allow_extra_pickup,
            )

            if plan:
                candidates.append((capacity, plan, cost))

        if not candidates:
            return {
                "plan": [],
                "total_cost": float("inf"),
            }

        if mode_str not in ["2", "3", "4"] and len(candidates) > 1:
            _, best_plan, best_cost = self._select_auto_candidate(candidates)
            return {
                "plan": best_plan,
                "total_cost": best_cost,
            }

        _, best_plan, best_cost = min(
            candidates,
            key=lambda candidate: (
                candidate[2],
                self._smoothness_score(candidate[1]),
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
        targets,
        r1_kfs,
        fake_walls,
        dynamic_obstacles,
        team,
        capacity,
        current_payload=0,
        visited_history=None,
        allow_extra_pickup=True,
    ):
        visited_history = visited_history or []
        r1_kfs = set(r1_kfs or [])

        targets = [
            target
            for target in targets
            if target not in r1_kfs
        ]

        all_obstacles = set(fake_walls) | set(dynamic_obstacles)
        entrance_targets = [
            target
            for target in targets
            if target in [1, 2, 3]
        ]

        best_plan = []
        min_cost = float("inf")
        best_smoothness = float("inf")

        for permutation in itertools.permutations(targets, capacity):
            if (
                current_node == 0
                and entrance_targets
                and permutation
                and permutation[0] not in entrance_targets
            ):
                continue

            unpicked = set(targets) - set(permutation)
            obstacles = set(all_obstacles) | set(unpicked)

            curr_pos = current_node
            curr_heading = current_heading
            builder = PlanBuilder()

            possible = True

            for index, target in enumerate(permutation):
                remaining_targets = permutation[index + 1:]

                if self._violates_last_pick_next_move_rule(
                    curr_pos=curr_pos,
                    curr_heading=curr_heading,
                    target=target,
                    remaining_targets=remaining_targets,
                    obstacles=obstacles,
                    team=team,
                    r1_kfs=r1_kfs,
                    visited_history=visited_history,
                    outside_front=builder.outside_front,
                ):
                    possible = False
                    break

                possible, curr_pos, curr_heading = self._collect_target(
                    builder=builder,
                    curr_pos=curr_pos,
                    curr_heading=curr_heading,
                    target=target,
                    obstacles=obstacles,
                    team=team,
                    r1_kfs=r1_kfs,
                    visited_history=visited_history,
                )

                if not possible:
                    break

                obstacles.discard(target)

                if builder.total_cost >= min_cost:
                    possible = False
                    break

            if not possible:
                continue

            payload_after_pick = current_payload + len(self._get_picked_targets(builder))

            exit_path, _ = self.search.find_path(
                start_node=curr_pos,
                start_heading=curr_heading,
                target_node=13,
                obstacles=obstacles,
                team=team,
                r1_kfs=r1_kfs,
                visited_history=visited_history,
                stand_and_reach=False,
                outside_front=builder.outside_front,
            )

            if exit_path:
                curr_pos, curr_heading = builder.extend_path(
                    path=exit_path,
                    curr_pos=curr_pos,
                    curr_heading=curr_heading,
                    team=team,
                    r1_kfs=r1_kfs,
                    visited_history=visited_history,
                )

            elif allow_extra_pickup and payload_after_pick < self.max_capacity:
                extra_result = self._try_extra_pickup_and_exit(
                    base_builder=builder,
                    curr_pos=curr_pos,
                    curr_heading=curr_heading,
                    unpicked=unpicked,
                    obstacles=obstacles,
                    payload_count=payload_after_pick,
                    team=team,
                    r1_kfs=r1_kfs,
                    visited_history=visited_history,
                )

                if extra_result is None:
                    continue

                builder, curr_pos, curr_heading = extra_result

            else:
                continue

            plan, cost = builder.result()
            smoothness = self._smoothness_score(plan)

            if DEBUG_ROUTE_EVALUATOR:
                print(
                    "[DEBUG ROUTE RESULT]",
                    f"perm={permutation}",
                    f"cost={cost:.2f}",
                    f"smooth={smoothness:.2f}",
                    f"picked={self._get_picked_targets(builder)}",
                )

            if (
                cost < min_cost - 1e-6
                or (
                    abs(cost - min_cost) <= 1e-6
                    and smoothness < best_smoothness
                )
            ):
                best_plan = plan
                min_cost = cost
                best_smoothness = smoothness

        return best_plan, min_cost

    def _collect_target(
        self,
        builder,
        curr_pos,
        curr_heading,
        target,
        obstacles,
        team,
        r1_kfs,
        visited_history,
    ):
        if target in r1_kfs:
            return False, curr_pos, curr_heading

        if self._is_target_picked(builder, target):
            return True, curr_pos, curr_heading

        if curr_pos == 0 and target in [1, 2, 3]:
            if builder.outside_front != target:
                curr_heading = builder.add_action(
                    curr_pos=0,
                    target=target,
                    curr_heading=curr_heading,
                    team=team,
                    action_type="APPROACH",
                )

            curr_heading = builder.add_action(
                curr_pos=0,
                target=target,
                curr_heading=curr_heading,
                team=team,
                action_type="PICK_UP",
            )

            if target in visited_history:
                builder.add_visit_penalty()

            return True, curr_pos, curr_heading

        path, _ = self.search.find_path(
            start_node=curr_pos,
            start_heading=curr_heading,
            target_node=target,
            obstacles=obstacles,
            team=team,
            r1_kfs=r1_kfs,
            visited_history=visited_history,
            stand_and_reach=True,
            outside_front=builder.outside_front,
        )

        if not path:
            return False, curr_pos, curr_heading

        curr_pos, curr_heading = builder.extend_path(
            path=path,
            curr_pos=curr_pos,
            curr_heading=curr_heading,
            team=team,
            r1_kfs=r1_kfs,
            visited_history=visited_history,
        )

        curr_heading = builder.add_action(
            curr_pos=curr_pos,
            target=target,
            curr_heading=curr_heading,
            team=team,
            action_type="PICK_UP",
        )

        return True, curr_pos, curr_heading

    def _try_extra_pickup_and_exit(
        self,
        base_builder,
        curr_pos,
        curr_heading,
        unpicked,
        obstacles,
        payload_count,
        team,
        r1_kfs,
        visited_history,
    ):
        if payload_count >= self.max_capacity:
            return None

        candidates = [
            target
            for target in unpicked
            if target not in r1_kfs
            and not self._is_target_picked(base_builder, target)
        ]

        if not candidates:
            return None

        best_result = None
        best_cost = float("inf")
        best_smoothness = float("inf")

        for target in candidates:
            candidate_builder = base_builder.clone()
            candidate_obstacles = set(obstacles)

            possible, sim_pos, sim_heading = self._collect_target(
                builder=candidate_builder,
                curr_pos=curr_pos,
                curr_heading=curr_heading,
                target=target,
                obstacles=candidate_obstacles,
                team=team,
                r1_kfs=r1_kfs,
                visited_history=visited_history,
            )

            if not possible:
                continue

            candidate_obstacles.discard(target)

            exit_path, _ = self.search.find_path(
                start_node=sim_pos,
                start_heading=sim_heading,
                target_node=13,
                obstacles=candidate_obstacles,
                team=team,
                r1_kfs=r1_kfs,
                visited_history=visited_history,
                stand_and_reach=False,
                outside_front=candidate_builder.outside_front,
            )

            if not exit_path:
                continue

            final_pos, final_heading = candidate_builder.extend_path(
                path=exit_path,
                curr_pos=sim_pos,
                curr_heading=sim_heading,
                team=team,
                r1_kfs=r1_kfs,
                visited_history=visited_history,
            )

            plan, cost = candidate_builder.result()
            smoothness = self._smoothness_score(plan)

            if (
                cost < best_cost - 1e-6
                or (
                    abs(cost - best_cost) <= 1e-6
                    and smoothness < best_smoothness
                )
            ):
                best_cost = cost
                best_smoothness = smoothness
                best_result = (
                    candidate_builder,
                    final_pos,
                    final_heading,
                )

        return best_result

    def _select_auto_candidate(self, candidates):
        by_capacity = {}

        for capacity, plan, cost in candidates:
            previous = by_capacity.get(capacity)

            if previous is None:
                by_capacity[capacity] = (plan, cost)
                continue

            previous_plan, previous_cost = previous

            if (
                cost < previous_cost - 1e-6
                or (
                    abs(cost - previous_cost) <= 1e-6
                    and self._smoothness_score(plan)
                    < self._smoothness_score(previous_plan)
                )
            ):
                by_capacity[capacity] = (plan, cost)

        capacities = sorted(by_capacity.keys())
        best_capacity = capacities[0]

        for capacity in capacities[1:]:
            current_plan, current_cost = by_capacity[best_capacity]
            next_plan, next_cost = by_capacity[capacity]

            threshold = self._auto_upgrade_threshold(
                from_capacity=best_capacity,
                to_capacity=capacity,
            )

            if (
                next_cost - current_cost <= threshold
                or next_cost < current_cost - 1e-6
            ):
                best_capacity = capacity

        best_plan, best_cost = by_capacity[best_capacity]
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

    def _violates_last_pick_next_move_rule(
        self,
        curr_pos,
        curr_heading,
        target,
        remaining_targets,
        obstacles,
        team,
        r1_kfs,
        visited_history,
        outside_front,
    ):
        if curr_pos == 0:
            return False

        if not self._can_pick_from_position(
            curr_pos=curr_pos,
            target=target,
            obstacles=obstacles,
        ):
            return False

        local_targets = [target]

        for other_target in remaining_targets:
            if self._can_pick_from_position(
                curr_pos=curr_pos,
                target=other_target,
                obstacles=obstacles,
            ):
                local_targets.append(other_target)

        if len(local_targets) <= 1:
            return False

        next_move = self._next_move_after_local_pickups(
            curr_pos=curr_pos,
            curr_heading=curr_heading,
            local_targets=local_targets,
            remaining_targets=remaining_targets,
            obstacles=obstacles,
            team=team,
            r1_kfs=r1_kfs,
            visited_history=visited_history,
            outside_front=outside_front,
        )

        if next_move is None:
            return False

        best_last_picks = self._best_last_picks_by_heading(
            curr_pos=curr_pos,
            local_targets=local_targets,
            next_move=next_move,
            team=team,
        )

        if len(best_last_picks) != 1:
            return False

        best_last_pick = next(iter(best_last_picks))

        if target == best_last_pick:
            other_local_targets_after_this = [
                node
                for node in local_targets
                if node != target
                and node in remaining_targets
            ]

            if other_local_targets_after_this:
                return True

        return False

    def _next_move_after_local_pickups(
        self,
        curr_pos,
        curr_heading,
        local_targets,
        remaining_targets,
        obstacles,
        team,
        r1_kfs,
        visited_history,
        outside_front,
    ):
        local_targets = set(local_targets)

        path_obstacles = set(obstacles)
        path_obstacles.difference_update(local_targets)

        next_target_after_local = None

        for target in remaining_targets:
            if target not in local_targets:
                next_target_after_local = target
                break

        if next_target_after_local is not None:
            path, _ = self.search.find_path(
                start_node=curr_pos,
                start_heading=curr_heading,
                target_node=next_target_after_local,
                obstacles=path_obstacles,
                team=team,
                r1_kfs=r1_kfs,
                visited_history=visited_history,
                stand_and_reach=True,
                outside_front=outside_front,
            )
        else:
            path, _ = self.search.find_path(
                start_node=curr_pos,
                start_heading=curr_heading,
                target_node=13,
                obstacles=path_obstacles,
                team=team,
                r1_kfs=r1_kfs,
                visited_history=visited_history,
                stand_and_reach=False,
                outside_front=outside_front,
            )

        if not path or len(path) < 2:
            return None

        return path[1]

    def _best_last_picks_by_heading(
        self,
        curr_pos,
        local_targets,
        next_move,
        team,
    ):
        scores = {}

        for target in local_targets:
            scores[target] = self._heading_alignment_score(
                curr_pos=curr_pos,
                pickup_target=target,
                next_move=next_move,
                team=team,
            )

        if not scores:
            return set()

        best_score = min(scores.values())

        return {
            target
            for target, score in scores.items()
            if score == best_score
        }

    def _heading_alignment_score(
        self,
        curr_pos,
        pickup_target,
        next_move,
        team,
    ):
        move_heading = KinematicsEngine.get_global_direction(
            curr_pos,
            next_move,
            team,
        )

        pickup_heading = KinematicsEngine.get_global_direction(
            curr_pos,
            pickup_target,
            team,
        )

        angle_diff = KinematicsEngine.normalize_angle(
            pickup_heading - move_heading
        )

        angle_diff = abs(angle_diff)

        if angle_diff > 180:
            angle_diff = 360 - angle_diff

        return angle_diff

    def _can_pick_from_position(
        self,
        curr_pos,
        target,
        obstacles,
    ):
        if target in [0, 13]:
            return False

        effective_obstacles = set(obstacles)
        effective_obstacles.discard(target)

        valid_stands = KinematicsEngine.get_neighbors(
            target,
            effective_obstacles,
        )

        valid_stands = [
            node
            for node in valid_stands
            if node not in [0, 13]
        ]

        return curr_pos in valid_stands

    def _smoothness_score(self, plan):
        if not plan:
            return float("inf")

        score = 0.0

        for index, action in enumerate(plan):
            action_type = action.get("action")

            if action_type == "ROTATE":
                score += 0.10

            if action_type == "PICK_UP":
                score += index * 0.01

            if action_type in ["MOVE", "EXIT"]:
                move_from = action.get("node")
                move_to = action.get("target")

                last_pickup = None

                for prev_index in range(index - 1, -1, -1):
                    previous = plan[prev_index]
                    previous_type = previous.get("action")

                    if (
                        previous_type == "PICK_UP"
                        and previous.get("node") == move_from
                    ):
                        last_pickup = previous
                        break

                    if previous_type in ["MOVE", "ENTER", "EXIT", "APPROACH"]:
                        break

                if last_pickup is not None:
                    if last_pickup.get("target") != move_to:
                        score += 1.0

        return score

    @staticmethod
    def _get_picked_targets(builder):
        plan, _ = builder.result()

        return {
            action.get("target")
            for action in plan
            if action.get("action") == "PICK_UP"
        }

    def _is_target_picked(self, builder, target):
        return target in self._get_picked_targets(builder)