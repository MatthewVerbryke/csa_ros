#!/usr/bin/env python3

"""
  Functions for working with a CSA "Response" type message.
  
  Copyright 2021 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


import rospy
from std_msgs.msg import Header

from csa_msgs.msg import Response


def create_response_msg(seq_num, src, dest, status, reject_msg):
    """
    Build a "Response" message from the input data.
    """
    
    # Create the message
    msg = Response()
    
    # Create the header
    msg.header = Header()
    msg.header.seq = seq_num
    msg.header.stamp = rospy.Time.now()
    
    # Build message from inputs
    msg.source = src
    msg.destination = dest
    msg.status = status
    msg.reject_msg = reject_msg
    
    return msg
