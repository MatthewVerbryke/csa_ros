#!/usr/bin/env python3

"""
  Functions for working with a CSA "Response" type message.
  
  Copyright 2021-2022 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/csa_ros
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


import rospy
from std_msgs.msg import Header

from csa_msgs.msg import Response
from csa_msgs.params import ParametersObj


def create_response_msg(id_num, src, dest, status, reject_msg, params, frame):
    """
    Build a "Response" message from input informatoin.
    """
    
    # Create the message
    msg = Response()
    
    # Create the header
    msg.header = Header()
    msg.header.stamp = rospy.Time.now()
    if frame is not None:
        msg.header.frame_id = frame
    
    # Build message from inputs
    msg.id = id_num
    msg.source = src
    msg.destination = dest
    msg.status = status
    msg.reject_msg = reject_msg
    
    # Handle None parameter input
    if params == None:
        pass
    else:
        msg.params = params
    
    return msg

def create_response_obj_from_msg(msg):
    """
    Create a 'ResponseObj' object directly from a 'Directive' ROS
    message.
    """
    
    obj = ResponseObj()
    obj.from_msg(msg)
    
    return obj

def create_response_obj(id_num, src, dest, status, reject_msg, params, frame):
    """
    Create a 'ResponseObj' object from from input information.
    """
    
    obj = ResponseObj()
    obj.from_values(id_num, src, dest, status, reject_msg, params, frame)
    
    return obj


class ResponseObj(object):
    """
    A Response object for CSA related code.
    """
    
    def __init__(self):
        
        # Initialize empty elements of Response
        self.header = Header()
        self.id = -1
        self.source = ""
        self.destination = ""
        self.status = ""
        self.reject_msg = ""
        self.params = ParametersObj()
    
    def __str__(self):
        
        s = "\n========== RESPONSE {} ==========\n\n".format(self.id)
        
        # Print out main values
        s += " - SOURCE MODULE: {}\n".format(self.source)
        s += " - DESTINATION MODULE: {}\n".format(self.destination)
        s += " - STATUS: {}\n".format(self.status)
        if self.status == "failure" or self.status == "reject":
            s += "{}\n".format(self.reject_message)
        else:
            s += "\n"
        
        # Print out parameters using its str function
        s += str(self.params)
        
        return s
    
    def from_msg(self, msg):
        """
        Populate the contents of the response object from a 'Response'
        ROS message.
        """
        
        # Store main response elements
        self.header = msg.header
        self.id = msg.id
        self.source = msg.source
        self.destination = msg.destination
        self.status = msg.status
        self.reject_msg = msg.reject_msg
        
        # Process and store message parameters
        self.params.from_msg(msg.params)
    
    def from_values(self, id_num, src, dest, status, reject_msg, params, frame):
        """
        Populate the contents of the response directly from the
        specified inputs.
        """
        
        # Store header elements
        if frame is not None:
            self.header.frame_id = frame
        
        # Store other information
        self.id = int(id_num)
        self.source = str(src)
        self.destination = str(dest)
        self.status = str(status)
        self.reject_msg = str(reject_msg)
        
        # Construct Parameter object from recognized input types
        if type(params) == dict:
            self.params.from_dicts(params)
        elif type(params) == ParametersObj:
            self.params = params
        elif params == None:
            pass
        else:
            print("Parameter input type {} not recognized".format(type(params)))
            print("Response {}, {} --> {}".format(id_num, src, dest))
    
    def to_msg(self):
        """
        Convert the response object to an equivalent `Response` ROS 
        message for publishing.
        """
        
        # Create the message
        msg = Response()
        
        # Build msg from information
        msg.header = self.header
        msg.header.stamp = rospy.Time.now()
        msg.id = self.id
        msg.source = self.source
        msg.destination = self.destination
        msg.status = self.status
        msg.reject_msg = self.reject_msg
        msg.params = self.params.to_msg()
        
        return msg

