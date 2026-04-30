##!/usr/bin/env python3

"""
  Functions for working with a CSA module status messages (actually of
  "DiagnosticStatus"-type) sub-message.
  
  Copyright 2021-2025 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/csa_ros
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


import rospy

from csa_msgs.msg import ModuleStatus
from csa_msgs.key_value import dict_to_key_value_list


def create_module_status_msg(module_name, start_t, end_t, directive, tactic_name,
                             act_list):
    """
    Build a status message from the input data, in the form of a 
    "ModuleStatus" message.
    """
    
    # Create message
    msg = ModuleStatus()
    
    # Build messages from inputs
    msg.name = module_name
    msg.start_t = start_t
    msg.end_t = end_t
    msg.cur_directive = directive
    msg.cur_tactic = tactic_name
    msg.cur_activities = act_list
    
    return msg
