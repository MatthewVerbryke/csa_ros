#!/usr/bin/env python3

"""
  Pass-through activity manager algorithm source code.
  
  Copyright 2023 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/csa_ros
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


import copy

import rospy

from csa_module.activity_manager_algorithm import ActivityManagerAlgorithm
from csa_msgs.msg import Directive, Response


class PassThroughActivityManager(ActivityManagerAlgorithm):
    """
    A very simple activity manager algorithm which simply passes through
    directives to be published with no response expected.
    
    NOTE: Only to be used when sending one directive per destination.
    """
    
    def __init__(self):
        
        # Set response expectation parameter
        expect_resp = False
        super().__init__(expect_resp)
        
    def process_response(self, response):
        """
        Simple return status and params of response
        """
        
        # Copy relevant response elements
        mode = response.status
        params = response.params
        
        return mode, params
        
    def execute_activity(self, directive):
        """
        Simply return the directives
        
        TODO: check to make sure we aren't overloaded?
        """
        
        directives_out = {}
        
        # Copy direcitve list
        id_num = directive.id
        
        # Package output
        directives_out.update({id_num: directive})
        
        return directives_out, True, ""
