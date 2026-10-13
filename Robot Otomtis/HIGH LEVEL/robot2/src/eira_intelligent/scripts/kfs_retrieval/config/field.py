HEIGHTS = {
    0: 0,    # luar depan forest
    13: 0,   # luar belakang forest
    21: 0,   # samping kiri forest
    22: 0,   # samping kanan forest

    2: 200,
    4: 200,
    10: 200,
    12: 200,

    1: 400,
    3: 400,
    5: 400,
    7: 400,
    9: 400,
    11: 400,

    6: 600,
    8: 600,
}


def build_adjacency():
    adj = {
        0: [1, 2, 3],
        13: [],
        21: [],
        22: [],
    }

    for node in range(1, 13):
        neighbors = []

        if node in [1, 2, 3]:
            neighbors.append(0)

        if node in [10, 11, 12]:
            neighbors.append(13)

        if node + 3 <= 12:
            neighbors.append(node + 3)

        if node - 3 >= 1:
            neighbors.append(node - 3)

        if node % 3 != 0:
            neighbors.append(node + 1)

        if node % 3 != 1:
            neighbors.append(node - 1)

        adj[node] = neighbors

    return adj


ADJACENCY = build_adjacency()