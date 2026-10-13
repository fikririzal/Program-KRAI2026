from ..config.field import HEIGHTS, ADJACENCY


class KinematicsEngine:
    ARENA_MIN_NODE = 1
    ARENA_MAX_NODE = 12
    ENTRANCE_NODES = [1, 2, 3]
    EXIT_NODES = [10, 11, 12]

    @staticmethod
    def get_neighbors(node, obstacles):
        if node == 13:
            return []

        return [
            neighbor
            for neighbor in ADJACENCY.get(node, [])
            if neighbor not in obstacles
        ]

    @staticmethod
    def get_height(node):
        return HEIGHTS.get(node, 0)

    @staticmethod
    def is_arena_node(node):
        try:
            value = int(node)
        except Exception:
            return False

        return (
            KinematicsEngine.ARENA_MIN_NODE
            <= value
            <= KinematicsEngine.ARENA_MAX_NODE
        )

    @staticmethod
    def is_red_team(team):
        return str(team or "").lower().strip() == "red"

    @staticmethod
    def is_entrance_node(node):
        return node in KinematicsEngine.ENTRANCE_NODES

    @staticmethod
    def is_exit_node(node):
        return node in KinematicsEngine.EXIT_NODES

    @staticmethod
    def is_outside_to_entrance(from_node, to_node):
        return from_node == 0 and to_node in KinematicsEngine.ENTRANCE_NODES

    @staticmethod
    def is_entrance_to_outside(from_node, to_node):
        return from_node in KinematicsEngine.ENTRANCE_NODES and to_node == 0

    @staticmethod
    def is_exit_to_outside(from_node, to_node):
        return from_node in KinematicsEngine.EXIT_NODES and to_node == 13

    @staticmethod
    def get_elevation_info(from_node, to_node):
        h_from = KinematicsEngine.get_height(from_node)
        h_to = KinematicsEngine.get_height(to_node)

        delta = abs(h_to - h_from)

        if h_to > h_from:
            direction = "UP"
        elif h_from > h_to:
            direction = "DOWN"
        else:
            direction = "FLAT"

        return direction, delta

    @staticmethod
    def get_pickup_elevation_info(stand_node, target_node):
        return KinematicsEngine.get_elevation_info(stand_node, target_node)

    @staticmethod
    def get_move_elevation_info(from_node, to_node):
        return KinematicsEngine.get_elevation_info(from_node, to_node)

    @staticmethod
    def normalize_angle(angle):
        result = (angle + 180) % 360 - 180

        if result == -180:
            return 180

        return result

    @staticmethod
    def get_relative_direction(angle_diff):
        normalized = KinematicsEngine.normalize_angle(angle_diff)

        if normalized == 0:
            return "Forward"

        if normalized == 90:
            return "Right"

        if normalized == -90:
            return "Left"

        if abs(normalized) == 180:
            return "Backward"

        if normalized > 0:
            return "Right"

        return "Left"

    @staticmethod
    def get_global_direction(src_node, dst_node, team):
        if src_node == 0:
            return 0

        if dst_node == 13:
            return 0

        if dst_node == 0:
            return 180

        if src_node == 13:
            return 180

        is_red = KinematicsEngine.is_red_team(team)

        if dst_node == 22:
            return -90 if is_red else 90

        if dst_node == 21:
            return 90 if is_red else -90

        diff = dst_node - src_node

        if diff == 3:
            return 0

        if diff == -3:
            return 180

        if diff == 1:
            return -90 if is_red else 90

        if diff == -1:
            return 90 if is_red else -90

        return 0

    @staticmethod
    def get_relative_direction_between(
        src_node,
        dst_node,
        curr_heading,
        team,
    ):
        global_dir = KinematicsEngine.get_global_direction(
            src_node,
            dst_node,
            team,
        )
        angle_diff = KinematicsEngine.normalize_angle(global_dir - curr_heading)

        return KinematicsEngine.get_relative_direction(angle_diff)

    @staticmethod
    def get_entrance_order(team):
        team_color = str(team).lower()

        if team_color == "blue":
            return [3, 2, 1]

        return [1, 2, 3]

    @staticmethod
    def get_approach_shift_info(team, from_front, to_front):
        if from_front is None:
            return "Forward", 0

        if from_front == to_front:
            return "NONE", 0

        order = KinematicsEngine.get_entrance_order(team)

        if from_front not in order or to_front not in order:
            return "Forward", 0

        from_idx = order.index(from_front)
        to_idx = order.index(to_front)
        delta = abs(to_idx - from_idx)

        if to_idx > from_idx:
            direction = "SHIFT_LEFT"
        else:
            direction = "SHIFT_RIGHT"

        return direction, delta

    @staticmethod
    def _wall_neighbor_candidates(node):
        candidates = list(ADJACENCY.get(node, []))

        if node in KinematicsEngine.ENTRANCE_NODES:
            candidates.append(0)

        if node in KinematicsEngine.EXIT_NODES:
            candidates.append(13)

        if node % 3 == 1:
            candidates.append(21)

        if node % 3 == 0:
            candidates.append(22)

        result = []

        for candidate in candidates:
            if candidate in result:
                continue

            if candidate in [0, 13, 21, 22]:
                result.append(candidate)
                continue

            if KinematicsEngine.is_arena_node(candidate):
                result.append(candidate)

        return result

    @staticmethod
    def get_neighbor_by_global_direction(node, global_direction, team=None):
        try:
            node = int(node)
        except Exception:
            return None

        if not KinematicsEngine.is_arena_node(node):
            return None

        target_direction = KinematicsEngine.normalize_angle(global_direction)

        for neighbor in KinematicsEngine._wall_neighbor_candidates(node):
            neighbor_direction = KinematicsEngine.get_global_direction(
                node,
                neighbor,
                team,
            )

            neighbor_direction = KinematicsEngine.normalize_angle(
                neighbor_direction
            )

            if neighbor_direction == target_direction:
                return neighbor

        return None

    @staticmethod
    def get_wall_value_by_elevation(from_node, to_node, team=None):
        if to_node is None:
            return 1

        elev, _ = KinematicsEngine.get_elevation_info(
            from_node,
            to_node,
        )

        if elev == "UP":
            return 1

        return 0

    @staticmethod
    def get_relative_wall_array(node, heading, team=None):
        try:
            node = int(node)
            heading = int(heading)
        except Exception:
            return [1, 1, 1, 1]

        if not KinematicsEngine.is_arena_node(node):
            return [0, 0, 0, 0]

        heading = KinematicsEngine.normalize_angle(heading)

        directions = [
            heading,
            KinematicsEngine.normalize_angle(heading - 90),
            KinematicsEngine.normalize_angle(heading + 90),
            KinematicsEngine.normalize_angle(heading + 180),
        ]

        wall = []

        for global_direction in directions:
            target_node = KinematicsEngine.get_neighbor_by_global_direction(
                node=node,
                global_direction=global_direction,
                team=team,
            )

            wall.append(
                KinematicsEngine.get_wall_value_by_elevation(
                    from_node=node,
                    to_node=target_node,
                    team=team,
                )
            )

        return wall

    @staticmethod
    def get_relative_wall_info(node, heading, team=None):
        wall = KinematicsEngine.get_relative_wall_array(
            node=node,
            heading=heading,
            team=team,
        )

        return {
            "front": int(wall[0]),
            "left": int(wall[1]),
            "right": int(wall[2]),
            "back": int(wall[3]),
        }

    @staticmethod
    def get_relative_wall_detail(node, heading, team=None):
        try:
            node = int(node)
            heading = int(heading)
        except Exception:
            return {}

        if not KinematicsEngine.is_arena_node(node):
            return {}

        heading = KinematicsEngine.normalize_angle(heading)

        relative_to_global = {
            "front": heading,
            "left": KinematicsEngine.normalize_angle(heading - 90),
            "right": KinematicsEngine.normalize_angle(heading + 90),
            "back": KinematicsEngine.normalize_angle(heading + 180),
        }

        detail = {}

        for relative_name, global_direction in relative_to_global.items():
            target_node = KinematicsEngine.get_neighbor_by_global_direction(
                node=node,
                global_direction=global_direction,
                team=team,
            )

            if target_node is None:
                elev = "BLOCKED"
                delta = 0
                wall = 1
            else:
                elev, delta = KinematicsEngine.get_elevation_info(
                    node,
                    target_node,
                )
                wall = KinematicsEngine.get_wall_value_by_elevation(
                    from_node=node,
                    to_node=target_node,
                    team=team,
                )

            detail[relative_name] = {
                "wall": int(wall),
                "global_direction": int(global_direction),
                "target": target_node,
                "elev": elev,
                "delta": int(delta),
            }

        return detail