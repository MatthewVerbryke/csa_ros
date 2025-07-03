#!/usr/bin/env python3

"""
  
  
  Copyright 2021-2025 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/csa_ros
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


import rospy

from diagnostic_msgs.msg import DiagnosticStatus
from key_value import dict_to_key_value_list


def create_status_msg(level, fail_msg, module_name, cur_id, module_stats):
    """
    Build a status message from the input data, in the form of a 
    "DiagnosticStatus" message.
    """
    
    # Create message
    msg = DiagnosticStatus()
    
    # Build message from inputs
    msg.status.level = level
    msg.status.name = module_name
    msg.status.message = fail_msg
    msg.status.hardware_id = id_num #NOTE: actually storing directive id
    
    # Handle key value list for other stats
    msg.status.values = dict_to_key_value_list(module_stats)
    
    return msg
