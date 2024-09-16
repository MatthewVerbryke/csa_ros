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

import csa_module.tactic import Tactic
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
