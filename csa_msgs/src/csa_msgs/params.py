#!/usr/bin/env python3

"""
  Functions for working with a CSA "Params" type sub-message.
  
  Copyright 2022 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/csa_ros
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


import rospy
from diagnostic_msgs.msg import KeyValue

from csa_msgs.msg import Parameters


def create_param_submsg(entry_conds, end_conds, rules, criteria, deadline):
    """
    Build a "Parameters" sub-message from the input data.
    """
    
    # Create sub-message
    msg = Parameters()
    
    # Build messages from inputs
    msg.entry_conds = dict_to_key_value_list(entry_conds)
    msg.end_conds = dict_to_key_value_list(end_conds)
    msg.rules = dict_to_key_value_list(rules)
    msg.criteria = dict_to_key_value_list(criteria)
    msg.deadline = deadline
    
    return msg
    
def rosmsg_to_key_value_list(msg_in):
    """
    Convert a ROS message into a ROS 'KeyValue' message, in which value 
    is the actual message, and key is the message type. Allows a kludgy 
    way to pass arbitrary messages using rosmsgs.
    
    NOTE: Contents (AKA the 'values') will be serialized into a string
          regardless of what type they are at the start. They will need
          to be converted back into there original type at destination
          of the message using the 'key' as its type w/ ast.literal_eval
          or similar.
    """
    
    key_value_list = []
    
    # Append each KeyValue making with properly formatting
    for key,value in msg_in.items():
        entry = KeyValue()
        entry.key = str(key)
        entry.value = str(value)
        key_value_list.append(entry)
        
    return key_value_list    
