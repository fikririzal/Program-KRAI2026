# ==========================================
# Physical cost matrix
# ==========================================

# Approach costs
C_APPROACH_INITIAL = {
    1: 2.00,
    2: 1.75,
    3: 1.50,
}
C_APPROACH_SHIFT_PER_NODE = 0.5

# Elevation costs
C_ELEV_UP_200 = 1.5
C_ELEV_UP_400 = 1.8
C_ELEV_DOWN_200 = 1.7
C_ELEV_DOWN_400 = 2.2

# Rotation costs
C_ROT_90 = 0.25
C_ROT_180 = 0.50

# KFS manipulation costs
C_PICK_ELEV_UP = 0.10
C_PICK_ELEV_DOWN = 0.30
C_PICK_DIR_FORWARD = 1.20

# Validation costs
C_VALIDATE_UP = 0.01
C_VALIDATE_DOWN = 0.01

C_VALIDATE_ROW_PENALTY = {
    0: 0.50,  # node 1, 2, 3
    1: 0.30,  # node 4, 5, 6
    2: 0.10,  # node 7, 8, 9
    3: 0.05,  # node 10, 11, 12
}

# Exit bias costs
C_EXIT_NODE_10 = 0.30
C_EXIT_NODE_11 = 0.50
C_EXIT_NODE_12 = 0.80

# Small penalty to reduce zig-zag
C_NODE_PENALTY = 0.2

# Emergency extra-pick threshold.
THRESHOLD_EXTRA_PICKUP_FOR_EXPENSIVE_EXIT = 2.0