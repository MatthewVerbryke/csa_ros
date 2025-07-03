#!/usr/bin/env python3

"""
  Functions for working with a CSA "Params" type sub-message.
  
  NOTE: These functions can be used as a kludge to pass parameters of 
        arbitrary ROS message-types using the CSA Directive/Response 
        format. See TODO on how to utilize them within CSA format modules.
  TODO: Improve upon this method or replace with a better way of doing
        this?
  
  Copyright 2022-2023 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/csa_ros
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


import rospy

from csa_msgs.msg import Parameters
from key_value import key_value_list_to_dict, dict_to_key_value_list


def create_param_submsg(entry_conds, end_conds, criteria, rules, deadline):
    """
    Build a 'Parameters' sub-message from the input data.
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
    
def convert_dict_to_params(param_dict):
    """
    Convert a parameter dictionary into a 'Parameters' sub-message.
    """
    
    # Split up input keys
    entry_conds = param_dict["entry_conds"]
    end_conds = param_dict["end_conds"]
    rules = param_dict["rules"]
    criteria = param_dict["criteria"]
    deadline = param_dict["deadline"]
    
    # build message
    msg = create_param_submsg(entry_conds, end_conds, criteria, rules, deadline)
    
    return msg

def convert_params_to_dict(param_in):
    """
    Convert a 'Parameters' message into a dict conposed of native Python
    types (except 'deadline' which remains 'std_msgs/Duration'.
    """
    
    # Convert elements of 'Parameters' into dicts
    entry_conds = key_value_list_to_dict(param_in.entry_conds)
    end_conds = key_value_list_to_dict(param_in.end_conds)
    rules = key_value_list_to_dict(param_in.rules)
    criteria = key_value_list_to_dict(param_in.criteria)
    deadline = param_in.deadline
    
    # Package these elements into dict
    param_dict = {"entry_conds": entry_conds,
                  "end_conds": end_conds,
                  "rules": rules,
                  "criteria": criteria,
                  "deadline": deadline}
    
    return param_dict
