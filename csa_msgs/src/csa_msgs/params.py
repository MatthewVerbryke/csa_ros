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

import ast

import rospy
from diagnostic_msgs.msg import KeyValue

from csa_msgs.msg import Parameters


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

def dict_to_key_value_list(msg_in):
    """
    Convert an arbitrary dict of ROS-message types into a list of ROS
    'KeyValue' messages. 
    """
    
    key_value_list = []
    
    # Append each KeyValue making with properly formatting
    for key,value in msg_in.items():
        entry = KeyValue()
        entry.key = str(key)
        entry.value = str(value)
        key_value_list.append(entry)
        
    return key_value_list
    
def key_value_list_to_dict(kv_list):
    """
    Convert a ROS 'KeyValue' list into a dict of native Python types.
    """
    
    dict_out = {}
    
    # Evaluate each key-value pair and store in output dictionary
    for entry in kv_list:
        
        try:
            interp_value = ast.literal_eval(entry.value)
        except ValueError:# TODO: Fix this: catches strings, but not ideal
            interp_value = entry.value
        
        dict_out.update({entry.key: interp_value})
        
    return dict_out
