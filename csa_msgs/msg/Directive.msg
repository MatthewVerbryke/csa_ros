# Directive Header (directive ID, time transmitted, frame of reference)
Header header

# Type of action to be taken
string action

# Action start time (if needed for coordination or some other reason)
stamp start_time

# Objective of the this directive 
#   Note: Not all of these will be filled at the same time.
string target
csa_msgs/TimedPoseArray target_poses
trajectory_msgs/JointTrajectory joint_trajectory


