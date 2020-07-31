# Response header (directive ID, time transmitted, frame of reference)
Header header

# Flag to say that a directive has been received
bool msg_received

# Responding module status with optional description
string status

# Values that potentially need to be transmitted back
#   Note: Not all of these will be filled at the same time.
string response
csa_msgs/TimedPoseArray response_poses
trajectory_msgs/JointTrajectory response_joint_trajectory
