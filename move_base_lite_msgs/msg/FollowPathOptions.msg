# Desired speed for the robot while following the path
float64 desired_speed

# Position tolerance for reaching the goal pose
# If left at 0.0, defaults of the controller are used.
float64 goal_pose_position_tolerance

# Angle tolerance for reaching the goal pose
# If left at 0.0, defaults of the controller are used.
float64 goal_pose_angle_tolerance

# Rotates to goal pose
bool rotate_to_goal_pose_orientation