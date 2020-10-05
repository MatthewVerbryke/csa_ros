#!/usr/bin/env python

"""
  Functions for direct packing/unpacking of 'csa_msgs' types into JSON
  format messages for use with ROSbridge.
  
  Copyright 2020 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


import sys

# Set path to catkin_ws source directory
file_dir = sys.path[0]
sys.path.append(file_dir + "/../../..")
from rse_dam.communication import packing


def pack_directive(msg):
    """
    Package a 'csa_msgs/Directive' message
    """
    
    # Prepare message info
    action = msg.action
    target = msg.target
    rules = msg.rules
    parameters = msg.parameters
    
    # Package complex sub-messages
    header = packing.pack_header(msg.header)
    start = packing.pack_time(msg.start_time)
    pose_targets = pack_timed_pose_array(msg.target_poses)
    trajectory = packing.pack_jointtrajectory(msg.joint_trajectory)
    end = packing.pack_time(msg.end_time)
    
    # Package entire message
    directive = {"header": header,
                 "action": action,
                 "start_time": start,
                 "target": target,
                 "target_poses": pose_targets,
                 "joint_trajectory": trajectory,
                 "rules": rules,
                 "parameters": parameters,
                 "end_time": end_time}
                  
    return directive
    
def pack_response(msg):
    """
    Package a 'csa_ros/Response' message.
    """
    
    # Prepare message info
    msg_recieved = msg.msg_recieved
    status = msg.status
    response = msg.status
    
    # Package complex sub-messages
    header = packing.pack_header(msg.header)
    poses = pack_timed_pose_array(msg.response_poses)
    trajectory = packing.pack_jointtrajectory(msg.response_joint_trajectory)
    
    # Package entire message
    response = {"header": header,
                "msg_recieved": msg_recieved,
                "status": status,
                "response": response,
                "response_poses": poses,
                "response_joint_trajectory": trajectory}
                
    return response
    
def pack_timed_pose_array(array):
    """
    Package a 'csa_msgs/TimedPoseArray' message.
    """
    
    # Package sub-messages
    header = packing.pack_header(msg.header)
    poses = packing.pack_pose_list(msg.poses)
    times = packing.pack_time_array(msg.time_from_start)
    
    timed_pose_array = {"header": header,
                        "poses": poses,
                        "time_from_start": times}
                        
    return timed_pose_array
