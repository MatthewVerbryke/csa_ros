#!/usr/bin/env python3

"""
  Functions for working with ROS 'KeyValue'-type messages, particularly 
  conversion between this type and native python dicts.
  
  Copyright 2021-2025 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/csa_ros
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


import ast

from diagnostic_msgs.msg import KeyValue


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
