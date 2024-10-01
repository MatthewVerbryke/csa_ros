#!/usr/bin/env python3

"""
  CSA module tactics component source code.
  
  Copyright 2021-2023 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/csa_ros
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


import rospy

from csa_common.inert import InertTactic
from csa_common.standby import StandbyTactic
from csa_msgs.msg import Parameters


class TacticsComponent(object):
    """
    A generic tactics component object for a CSA module.
        
    Selects a tactic (or series of tactics) to accomplish the current
    directive.
        - Recieves arbitrated directive and state information from the
          control component
        - Selects and sets-up the appropriate tactic (or tactics) for 
          the given information
        - Returns this tactic to the control component
    """
    
    def __init__(self, module_name, tactics_algorithm, rate, latency):
        
        # Parameters
        self.tactics_algorithm = tactics_algorithm  
        self.rate = rate
        self.latency = latency
        
        # Give relevant parameters to tactic algorithm
        self.tactics_algorithm.get_module_info(module_name, rate, latency)
        
    def run(self, directive, state, model):
        """
        Run the component once, reading in messages from the other
        components and performing its function.
        """
        
        # If module is standing by, quickly get Standby tactic
        if directive.name == "standby":
            name = self.tactics_algorithm.module_name
            params = {"rules": {"id":directive.id}}
            tactic = StandbyTactic(name, params, model)
            success = True
        
        # If commanded to go inert, quickly get Inert tactic
        elif directive.name == "inert":
            name = self.tactics_algorithm.module_name
            params = {"rules": {"id":directive.id}}
            tactic = InertTactic(name, params, model)
            success = True
        
        # Handle a new tactic request
        else:
            success, tactic = self.tactics_algorithm.run(directive, state, model)
        
        return success, tactic #<-- TODO: change to send multiple tactics
