#!/usr/bin/env python

"""
  CSA module tactics component source code.
  
  Copyright 2021 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/gazebo_terrain
  Additional copyright may be held by others, as reflected in the commit
  history.
  
  TODO: Test
"""


import rospy


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
    
    def __init__(self, tactics_algorithm):
        
        # Variables
        self.tactics_algorithm = tactics_algorithm        
        
    def run(self, directive, state):
        """
        Run the component once, reading in messages from the other
        components and performing its function.
        """
        
        # Handle a new tactic request
        if self.request_recieved:
            tactic = self.tactics_algorithm.run(directive, state)
            
            return tactic[0] #<-- TODO: change to send multiple tactics
        else:
            return None
