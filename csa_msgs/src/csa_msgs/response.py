#!/usr/bin/env python3

"""
  Functions for working with a CSA "Response" type message.
  
  Copyright 2021-2022 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/csa_ros
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


import rospy
from std_msgs.msg import Header

from csa_msgs.msg import Response


def create_response_msg(id_num, src, dest, status, reject_msg, params, frame):
    """
    Build a "Response" message from the input data.
    """
    
    # Create the message
    msg = Response()
    
    # Create the header
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()
    if frame is not None:
        msg.header.frame_id = frame
    
    # Build message from inputs
    msg.id = id_num
    msg.source = src
    msg.destination = dest
    msg.status = status
    msg.reject_msg = reject_msg
    msg.params = params
    
    return msg
