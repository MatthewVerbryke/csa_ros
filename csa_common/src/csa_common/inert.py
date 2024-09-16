#!/usr/bin/env python3

"""
  Default 'Inert' tactic source code.
  
  Copyright 2024 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/csa_ros
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


import rospy

from csa_module.tactic import Tactic
from csa_msgs.directive import create_directive_msg
from csa_msgs.params import create_param_submsg, convert_params_to_dict


class InertTactic(Tactic):
    """
    A tactic to inactivate a CSA module, so no commands are sent out.
    
    NOTE: different from 'Stop' tactics, which order system to stop at 
    current position.
    
    TODO: Test
    """
    
    def __init__(self, module_name, params, model):
        
        # Initialize parent class
        super().__init__(module_name, params, model)
        
        # Set as non-continous tactic
        self.continuous = False
    
    def run(self, state):
        
        # Simply create empty directive asking for stop
        params = create_param_submsg({}, {}, {}, {}, rospy.Time(0.0))
        directive = create_directive_msg(self.cur_id, "inert", "",
                                         self.module_name, "", 0.0, 1,
                                         params, "")
        success = True #TODO: handle success/failure consideration here?
        
        return success, directive


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
        return "continue"
