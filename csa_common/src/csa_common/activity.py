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


class InertActivity(Activity):
    """
    An Activity object for rendering all commanded modules inert; should
    be included by default in each new 'DiscreteActivityManager'
    
    TODO: Test
    """
    
    def __init__(self, dest_names):
        
        # Give name to activity
        name = "inert"
        super().__init__(name, dest_names)
        
        # Store common directive elements
        self.resp_t = 1.0
    
    def get_outputs(self, params):
        
        acts_out = []
        
        # create empty params message
        # TODO: will there ever be param inputs here?
        params = create_param_submsg({}, {}, {}, {}, rospy.Time(0.0))
        
        # Create directive for all commanded modules
        for name in self.dests:
            activity = create_directive_msg(-1, "inert", "", self.source, name,
                                            self.resp_t, 0, params, None) 
            acts_out.append(activity)
        
        return acts_out
    
    def check_response(self, response):
        
        # Activity only ended by command from above
        return continue
