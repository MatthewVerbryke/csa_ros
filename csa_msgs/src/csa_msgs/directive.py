#!/usr/bin/env python3

"""
  Functions for working with a CSA "Directive" type message.
  
  Copyright 2021-2025 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/csa_ros
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


import rospy
from std_msgs.msg import Header

from csa_msgs.msg import Directive
from params import ParameterObj


def create_directive_msg(id_num, name, desc, src, dest, t_resp, priority, params,
                         frame):
    """
    Build a "Directive" message from the input data.
    """
    
    # Create the message
    msg = Directive()
    
    # Create the header
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()
    if frame is not None:
        msg.header.frame_id = frame
    
    # Build message from inputs
    msg.id = id_num
    msg.name = name
    msg.description = desc
    msg.source = src
    msg.destination = dest
    msg.response_time = rospy.Time.now() + rospy.Duration(t_resp)
    msg.priority = priority
    
    # Handle None parameter input
    if params == None:
        pass
    else:
        msg.params = params
    
    return msg

class DirectiveObj(object):
    """
    A Directive object for CSA related code.
    """
    
    def __init__(self):
        
        # Initialize empty elements of Directive
        self.header = Header()
        self.id = -1
        self.name = ""
        self.description = ""
        self.source = ""
        self.destination = ""
        self.response_time = rospy.Duration(0.0)
        self.priority = -1
        self.params = ParameterObj()
    
    def __str__(self):
        
        s += "\n========== DIRECTIVE {}: '{}' ==========\n\n".format(
            self.id, self.name
        )
        
        # Print out main values
        s += " - SOURCE MODULE: {}\n".format(self.source)
        s += " - DESTINATION MODULE: {}\n".format(self.destination)
        s += " - RESPONSE TIME: {}\n".format(self.response_time)
        s += " - PRIORITY: {}\n".format(self.priority)
        s += " - REF. FRAME: {}\n\n".format(self.header.frame)
        
        # Print out parameters using its str function
        s += str(self.params)
        
        return s
    
    def from_msg(self, msg):
        """
        Populate the contents of the directive object from a 'Directive'
        ROS message.
        """
        
        # Store main directive elements
        self.header = msg.header
        self.id = msg.id
        self.name = msg.name
        self.description = msg.description
        self.source = msg.source
        self.destination = msg.destination
        self.response_time = msg.response_time
        self.priority = msg.priority
        
        # Process and store message parameters
        self.params.from_msg(msg.params)
        
    def from_values(self, id_num, name, desc, src, dest, t_resp, priority,
                    params, frame):
        """
        Populate the contents of the directive directly from the
        specified inputs.
        """
        
        # Store header elements
        if frame is not None:
            self.header.frame_id = frame
        
        # Store other information
        self.id = int(id_num)
        self.name = str(name)
        self.description = str(desc)
        self.source = str(src)
        self.destination = str(dest)
        self.response_time = rospy.Duration(t_resp)
        self.priority = int(priority)
        
        # Construct Parameter object from recognized input types
        if type(params) == dict:
            self.params.from_dicts(params)
        elif type(params) == ParameterObj()
            self.params = params
        else:
            print("Parameter input type {} not recognized".format(type(params))
    
    def to_msg(self):
        """
        Convert the directive object to an equivalent ROS message for 
        publishing.
        """
        
        # Create the message
        msg = Directive()
        
        # Build msg from information
        msg.header = self.header
        msg.header.stamp = rospy.Time.now()
        msg.id = self.id
        msg.name = self.name
        msg.description = self.description
        msg.source = self.source
        msg.destination = self.destination
        msg.response_time = self.response_time
        msg.priority = self.priority
        msg.params = self.params
        
        return msg

