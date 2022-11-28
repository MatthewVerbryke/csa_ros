#!/usr/bin/env python3

"""
  Functions for working with a CSA "Directive" type message.
  
  Copyright 2021-2022 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/csa_ros
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


import rospy
from std_msgs.msg import Header

from csa_msgs.msg import Directive


def create_directive_msg(id_num, name, desc, src, dest, t_resp, priority, params,
                         frame):
    """
    Build a "Directive" message from the input data.
    """
    
    # Create the message
    msg = Directive()
    
    # Create the header
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()
    if frame is not None:
        msg.header.frame_id = frame
    
    # Build message from inputs
    msg.id = id_num
    msg.name = name
    msg.description = desc
    msg.source = src
    msg.destination = dest
    msg.response_time = t_resp
    msg.priority = priority
    
    # Handle None parameter input
    if params == None:
        pass
    else:
        msg.params = params
    
    return msg
