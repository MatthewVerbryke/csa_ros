#!/usr/bin/env python3

"""
  Generic action base class source code.
  
  Copyright 2024 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/csa_ros
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


import rospy


class Activity(object):
    """
    Base class object for activities. Intended for use with the
    'DiscreteActivityManager' class in the common applications package.
    """
    
    def __init__(self, name, dests):
        
        self.name = name
        self.dests = dests
        self.source = ""
    
    def get_outputs(self, params):
        """
        Given parameters for the activity, return a directive or set of
        directives to perform the activity.
        """
        
        acts_out = []
        
        return acts_out
    
    def check_response(self, response):
        """
        Check a response to an output directive to determine if the
        activity is finished or should continue to execute.
        """
        
        return "failure"
