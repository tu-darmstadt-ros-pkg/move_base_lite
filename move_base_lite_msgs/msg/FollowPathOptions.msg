# Desired speed for the robot while following the path
# If left at 0.0, defaults of the controller are used.
float64 desired_speed

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