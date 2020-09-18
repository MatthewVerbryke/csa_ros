# Directive Header (directive ID, time transmitted, frame of reference)
Header header
#
# Type of action to be taken
string action
#
# Action start time
stamp start_time
#
# Objective of the this directive. Includes multiple ways to express it, so not all of these will be filled at the same time.
string target
csa_msgs/TimedPoseArray target_poses
trajectory_msgs/JointTrajectory joint_trajectory
#
# rules or parameters for action execution (best way to currently do this is to pass a dictionary as a string and convert back at destination)
string rules
stirng parameters
#
# Desired time of completion
stamp end_time
