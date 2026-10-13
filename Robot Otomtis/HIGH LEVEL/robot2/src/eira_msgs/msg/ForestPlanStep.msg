int32 NO_NODE=-1

int32 ACTION_UNKNOWN=0
int32 ACTION_APPROACH=1
int32 ACTION_ENTER=2
int32 ACTION_MOVE=3
int32 ACTION_ROTATE=4
int32 ACTION_PICK_UP=5
int32 ACTION_VALIDATE=6
int32 ACTION_EXIT=7

int32 step

int32 node
int32 target

int32 action
string action_name

string elev
int32 delta

string direction
int32 heading

# Wall info relative to robot heading
# wall[0] = front
# wall[1] = left
# wall[2] = right
# wall[3] = back
# value:
# 0 = tidak ada wall
# 1 = ada wall
int8[4] wall

float64 cost