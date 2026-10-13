import itertools


class R1PathwayPlanner:
    """
    Simple R1 recommendation planner.
    """

    def __init__(self):
        self.coords = {
            1: (0, 3),
            2: (1, 3),
            3: (2, 3),

            4: (0, 2),
            6: (2, 2),

            7: (0, 1),
            9: (2, 1),

            10: (0, 0),
            11: (1, 0),
            12: (2, 0),

            0: (1, 3.5),
            13: (1, -0.5),
        }

    def _calc_dist(self, pos1, pos2):
        p1 = self.coords.get(pos1, (1, 3))
        p2 = self.coords.get(pos2, (1, 3))

        return abs(p1[0] - p2[0]) + abs(p1[1] - p2[1])

    def get_recommendation(self, r1_targets, r2_plan, current_r1_pos=0):
        if not r1_targets:
            return []

        r2_validation_needs = []
        seen = set()

        for action in r2_plan:
            if (
                action["action"] == "VALIDATE"
                and action["target"] in r1_targets
                and action["target"] not in seen
            ):
                r2_validation_needs.append(action["target"])
                seen.add(action["target"])

        best_sequence = []
        min_cost = float("inf")

        for sequence in itertools.permutations(r1_targets):
            travel_distance = 0
            current = current_r1_pos

            for target in sequence:
                travel_distance += self._calc_dist(current, target)
                current = target

            travel_distance += self._calc_dist(current, 13)

            sync_penalty = 0

            for idx, needed_node in enumerate(r2_validation_needs):
                if needed_node in sequence and sequence.index(needed_node) < idx:
                    sync_penalty += 50

            total_cost = travel_distance + sync_penalty

            if total_cost < min_cost:
                min_cost = total_cost
                best_sequence = list(sequence)

        return best_sequence