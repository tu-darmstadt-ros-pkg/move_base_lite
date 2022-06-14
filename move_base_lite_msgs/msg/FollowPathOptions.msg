# Desired speed for the robot while following the path
# If left at 0.0, defaults of the controller are used.
float64 desired_speed

# If false, target path can be smoothed out
bool is_fixed

# Position tolerance for reaching the goal pose
# If left at 0.0, defaults of the controller are used.
float64 goal_pose_position_tolerance

# Angle tolerance for reaching the goal pose
# If left at 0.0, defaults of the controller are used.
float64 goal_pose_angle_tolerance

# Rotates to goal pose exactly if set to true. Otherwise,
# robot alignes either front or rear with the target pose,
# depending on which direction is shorter
bool rotate_front_to_goal_pose_orientation

# If true, robot is allowed to follow path backwards
# even if robot is not y-symmetric
bool reverse_allowed

# If true, robot is not allowed to drive forwards
# Only works if reverse_allowed is true as well
bool reverse_forced

# If true, the robot uses the orientation of path poses to determine
# driving direction. Only applies if driving in reverse is allowed.
bool use_path_orientation

# If true, resets the pose history used for stuck detection
# to prevent getting stuck immediately
bool reset_stuck_history
