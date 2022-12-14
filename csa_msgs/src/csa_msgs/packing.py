#!/usr/bin/env python3

"""
  Functions for direct packing/unpacking of 'csa_msgs' types into JSON
  format messages for use with ROSbridge.
  
  Copyright 2020-2022 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/rse_dam
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


import os
import sys


def pack_directive(directive):
    """
    Package a 'csa_msgs/Directive' message
    """
    
    # Prepare message info
    header = pack_header(directive.header)
    id_num = directive.id
    name = directive.name
    desc = directive.description
    source = directive.source
    dest = directive.destination
    r_time = pack_duration(directive.response_time)
    priority = directive.priority
    params = pack_params(directive.params)
    
    # Package entire message into dict
    directive_msg = {"header": header,
                     "id": id_name,
                     "name": name,
                     "description": desc,
                     "source": source,
                     "destination": dest,
                     "response_time": r_time,
                     "priority": priority,
                     "params": params}
    
    return directive_msg
    
def pack_response(response):
    """
    Package a 'csa_ros/Response' message.
    """
    
    # Prepare message info
    header = pack_header(response.header)
    id_num = response.id
    source = response.source
    dest = response.destination
    status = response.status
    reject_msg = response.reject_msg
    params = pack_params(response.params)
    
    # Package entire message
    response_msg = {"header": header,
                    "id": id_num,
                    "source": source,
                    "destination": dest,
                    "status": status,
                    "reject_msg": reject_msg,
                    "params": params}
    
    return response_msg
    
def pack_params(params):
    """
    Package a 'csa_ros/Parameters' message.
    """
    
    # Prepare message info
    entry_conds = pack_key_value(params.entry_conds)
    end_conds = pack_key_value(params.end_conds)
    rules = pack_key_value(params.rules)
    criteria = pack_key_value(params.criteria)
    deadline = pack_time(params.time)
    
    # Package entire message
    param_msg = {"entry_conds": entry_conds,
                 "end_conds": end_conds,
                 "rules": rules,
                 "criteria": criteria,
                 "deadline", deadline}
    
    return param_msg
