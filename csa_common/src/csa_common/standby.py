#!/usr/bin/env python3

"""
  Default 'Standby' tactic source code.
  
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


class StandbyTactic(Tactic):
    """
    A tactic for a CSA module to standby for further commands. Should be
    default Tactic for most modules.
    
    NOTE: different from 'Stop' tactics, which order system to stop at 
    current position.
    """
    
    def __init__(self, module_name, params, model):
        
        # Initialize parent class
        super().__init__(module_name, params, model)
        
        # Set as non-continous tactic
        self.continuous = False
        
    def run(self, state):
        
        # Simply don't return anything
        directive = None
        success = True
        
        return success, directive
        
