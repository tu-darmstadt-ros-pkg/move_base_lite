int32 val

# overall behavior
int32 SUCCESS=1
int32 FAILURE=99999

# Planner failed to generate plan
int32 PLANNING_FAILED=-1

# Plan generated, but not valid
int32 INVALID_MOTION_PLAN=-2

# Unable to follow path within thresholds
int32 CONTROL_FAILED=-4

# Preempted before reaching final state
int32 PREEMPTED=-7

# Stuck detection triggered
int32 STUCK_DETECTED = -8

# Tf failure
int32 TF_LOOKUP_FAILURE = -9


# planning & kinematics request errors
int32 START_STATE_IN_COLLISION=-10
int32 GOAL_IN_COLLISION=-12