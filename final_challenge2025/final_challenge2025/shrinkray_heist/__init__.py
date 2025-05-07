from enum import Enum, auto

class HeistState(Enum):
    IDLE = auto()
    PLAN_TRAJ = auto()
    FOLLOW_TRAJ = auto()
    WAIT_TRAFFIC = auto()
    SCOUT = auto()
    PARK = auto()
    PICKUP = auto()
    ESCAPE = auto()
    COMPLETE = auto()

HEISTSTATE = HeistState.IDLE