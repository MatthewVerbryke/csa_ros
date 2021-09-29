#!/usr/bin/env python

"""
  Functions for working with a CSA "Directive" type message.
  
  Copyright 2021 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


import rospy
from std_msgs.msg import Header

from csa_msgs.msg import Directive


def create_directive_msg(id_num, name, desc, src, dest, t_resp, priority, params):
    """
    Build a "Directive" message from the input data.
    
    TODO: Figure out how we will handle directives
    """
    
    # Create the message
    msg = Directive()
    
    # Create the header
    msg.header = Header()
    msg.header.seq = id_num
    msg.header.stamp = rospy.Time.now()
    
    # Build message from inputs
    msg.name = name
    msg.description = desc
    msg.source = src
    msg.destination = dest
    msg.response_time = t_resp
    msg.priority = priority
    #msg.params = Params() #TODO: Update this
    
    return msg
