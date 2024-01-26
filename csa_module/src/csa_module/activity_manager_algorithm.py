#!/usr/bin/env python3

"""
  Generic CSA activity manager algorithm base class source code.
  
  Copyright 2021-2024 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/gazebo_terrain
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


import rospy

from csa_msgs.msg import Directive, Response
from csa_msgs.response import create_response_msg
from csa_msgs.directive import create_directive_msg


class ActivityManagerAlgorithm(object):
    """
    Base class object for activitiy manager algorithms.
    """
    
    def __init__(self, expect_resp):
        self.expect_resp = expect_resp
    
    def process_response(self, response):
        
        mode = "failure"
        params = None
        
        return mode, params
    
    def execute_activity(self, directives):
        
        directives = []
        success = False
        msg = ""
        
        return directives, success, msg
