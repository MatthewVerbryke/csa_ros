# Response header (directive ID, time transmitted, frame of reference)
Header header
#
# Flag to say that a directive has been received
bool msg_received
#
# Responding module status with optional description
string status
string response
#
# Values that potentially need to be transmitted back. Not all of these will be filled at the same time.
csa_msgs/TimedPoseArray response_poses
trajectory_msgs/JointTrajectory response_joint_trajectory
