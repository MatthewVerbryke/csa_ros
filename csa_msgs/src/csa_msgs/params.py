#!/usr/bin/env python3

"""
  Functions for working with CSA "Params" type sub-message and 
  derivatives.
  
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
from std_msgs.msg import Time

from csa_msgs.msg import Parameters
from csa_msgs.key_value import key_value_list_to_dict, dict_to_key_value_list
from rosbridge_packing.packing import (
    get_ros_type, pack_ros_msg, unpack_ros_msg, get_ros_type_list
)


def create_param_submsg(entry_conds, end_conds, criteria, rules, deadline):
    """
    Build a 'Parameters' sub-message from input information.
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
    Convert a 'Parameters' sub-message into a dict conposed of native 
    Python types, except 'deadline' which remains 'std_msgs/Duration'.
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

def get_param_type(param):
    """
    Determine the type of the parameter including base Python types and
    ROS types which have conversion functions.
    
    NOTE: If your parameter types is not found, you may need to add it 
          to the list below (or the 'get_ros_type' function if its a ROS
          type)
    """
    
    msg = ""
    
    # Check for native Python types
    if type(param) == float:
        type_out = "float"
    elif type(param) == int:
        type_out = "int"
    elif type(param) == str:
        type_out = "str"
    elif type(param) == bool:
        type_out = "bool"
    elif type(param) == dict:
        type_out = "dict"
    elif type(param) == list:
        type_out = "list"
    elif type(param) == tuple:
        type_out = "tuple"
    
    # Check for ROS types
    else:
        ros_type = get_ros_type(param)
        if ros_type is not None:
            type_out = ros_type
        else:
            msg = "Parameter type {} not recognized"
    
    return type_out, msg

def pack_parameter(param, param_type):
    """
    Given a parameter and its type, 'package' it by converting it to 
    native Python types if not already one.
    """
    
    # Get names of all recognized types
    std_types = ["float", "int", "str", "bool", "dict", "list", "tuple"]
    ros_types = get_ros_type_list()
    
    # Perform approapriate action
    if param_type in std_types:
        packed_param = param
    elif param_type in ros_types:
        packed_param = pack_ros_msg(param)
    else:
        packed_param = None
    
    return packed_param
    
def unpack_parameter(packed_param, param_type):
    """
    Given a packaged parameter and its intended type, unpackage it to
    this type if required.
    """
    
    # Get names of all recognized types
    std_types = ["float", "int", "str", "bool", "dict", "list", "tuple"]
    ros_types = get_ros_type_list()
    
    # Perform approapriate action
    if param_type in std_types:
        param = packed_param
    elif param_type in ros_types:
        param = unpack_ros_msg(packed_param, param_type)
    else:
        param = None
    
    return param

def create_param_obj(values, objects, entry_conds, end_conds, criteria,
                     rules, deadline):
    """
    Cretae a 'ParametersObj' object from input information.
    """
    
    param_obj = ParametersObj()
    param_obj.from_dicts(
        values, objects, entry_conds, end_conds, criteria, rules, deadline
    )
    
    return param_obj


class ParametersObj(object):
    """
    An object to allow for greater encapsulation and flexibility when 
    dealing with parameters in CSA.
    """
    
    def __init__(self):
        
        # Initialize empty parameter holding dicts
        self.values = {}
        self.objects = {}
        self.entry_conds = {}
        self.end_conds = {}
        self.rules = {}
        self.criteria = {}
        self.deadline = rospy.Time(0.0)
        self.types = {}
    
    def __str__(self):
        
        s = "-------------------------\nVALUES:\n\n"
        for key,value in self.values.items():
            s += "'{}' (TYPE: '{}'):\n".format(key, self.types[key])
            if self.types[key] == "str":
                s += "'{}'\n\n".format(value)
            else:
                s += "{}\n\n".format(value)
        
        #TODO
        s += "-------------------------\nINITIAL CONDITIONS:\n\n"
        for value in self.entry_conds:
            s += "    ({})\n".format(value)
        
        #TODO
        s += "-------------------------\nGOAL CONDITIONS:\n\n"
        for value in self.end_conds:
            s += "    ({})\n".format(value)
        
        #TODO
        s += "-------------------------\nRULES:\n\n"
        for value in self.rules:
            s += "    ({})\n".format(value)
        
        #TODO
        s += "-------------------------\nCRITERIA:\n\n"
        for value in self.rules:
            s += "    ({})\n".format(value)
        
        s += "-------------------------\nDEADLINE: {}\n\n".format(
            self.deadline.secs + 0.000000001*self.deadline.nsecs)
        
        return s
    
    def retrieve_value_types(self, param_dict):
        """
        For raw parameters of a subset of the overall parameters (stored
        in a dictionary), store the name and type of the parameter in 
        the values dict if not already there.
        """
        
        # Make sure dict is not empty
        if not param_dict:
            pass
        
        # Store name and type of parameter
        else:
            for key,value in param_dict.items():
                if key not in self.types.keys():
                    name = key
                    type_str, msg = get_param_type(value)
                    self.types.update({name: type_str})
                else:
                    pass
    
    def from_msg(self, msg):
        """
        Populate the object from a 'Parameter' ROS message.
        """
        
        # Extract parameter types
        for entry in msg.types:
            self.types.update({entry.key: entry.value})
        
        # Convert values to dict and unpack each
        values_raw = key_value_list_to_dict(msg.values, self.types)
        for key,value in values_raw.items():
            value_unpack = unpack_parameter(value, self.types[key])
            self.values.update({key: value_unpack})
        
        # Store other parameters
        self.objects = key_value_list_to_dict(msg.objects)
        self.entry_conds = key_value_list_to_dict(msg.entry_conds)
        self.end_conds = key_value_list_to_dict(msg.end_conds)
        self.rules = key_value_list_to_dict(msg.rules)
        self.criteria = key_value_list_to_dict(msg.criteria)
        self.deadline = msg.deadline
    
    def from_dict(self, param_dicts):
        """
        Given all relevant parameters as a dictionary, store them in the
        proper locations and construct the 'values' entry from the names
        and types of each parameter.
        """
        
        # Determine types of parameters
        self.retrieve_value_types(param_dicts["values"])
        
        # Store actual parameters
        self.values = param_dicts["values"]
        self.objects = param_dicts["objects"]
        self.entry_conds = param_dicts["entry_conds"]
        self.end_conds = param_dicts["end_conds"]
        self.rules = param_dicts["rules"]
        self.criteria = param_dicts["criteria"]
        
        # Store deadline as proper type
        if type(param_dicts["deadline"]) == float:
            self.deadline = rospy.Time(param_dicts["deadline"])
        elif type(param_dicts["deadline"]) == Time:
            self.deadline = param_dicts["deadline"]
        elif param_dicts["deadline"] is None:
            self.deadline = rospy.Time(0.0)
    
    def from_dicts(self, values, objects, entry_conds, end_conds, criteria,
                   rules, deadline):
        """
        Given all relevant parameters as multiple dictionaries, store 
        them in the proper locations and construct the types dictionary.
        """
        
        # Determine types of parameters
        self.retrieve_value_types(values)
        
        # Store parameters
        self.values = values
        self.objects = objects
        self.entry_conds = entry_conds
        self.end_conds = end_conds
        self.rules = rules
        self.criteria = criteria
        
        # Store deadline as proper type
        if type(deadline) == float:
            self.deadline = rospy.Time(deadline)
        elif type(deadline) == Time:
            self.deadline = deadline
        elif deadline is None:
            self.deadline = rospy.Time(0.0)
        
    def to_msg(self):
        """
        Convert the parameter object to a 'Parameters' ROS message 
        (typically used as a sub-message within an overall 'Directive'
        message).
        """
        
        msg = Parameters()
        values_packed = {}
        
        # Package parameter values
        for key,value in self.values.items():
            val_packed = pack_parameter(value, self.types[key])
            values_packed.update({key: val_packed})
        
        # Create sub-message and add elements as key-value lists
        msg.values = dict_to_key_value_list(values_packed)
        msg.objects = dict_to_key_value_list(self.objects)
        msg.types = dict_to_key_value_list(self.types)
        msg.entry_conds = dict_to_key_value_list(self.entry_conds)
        msg.end_conds = dict_to_key_value_list(self.end_conds)
        msg.rules = dict_to_key_value_list(self.rules)
        msg.criteria = dict_to_key_value_list(self.criteria)
        msg.deadline = self.deadline
        
        return msg
