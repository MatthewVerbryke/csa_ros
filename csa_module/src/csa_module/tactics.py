#!/usr/bin/env python

"""
  CSA module tactics component source code.
  
  Copyright 2021 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/gazebo_terrain
  Additional copyright may be held by others, as reflected in the commit
  history.
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
    
    TODO: Test member functions.
    """
    
    def __init__(self, tactics_algorithm):
        
        # Flags
        self.request_recieved = False
        
        # Variables
        self.tactics_algorithm = tactics_algorithm        
        
    def run(self, ctrl_msg):
        """
        Run the component once, reading in messages from the other
        components and performing its function.
        """
        
        # Set flags for the current loop
        self.request_recieved = False
        
        # Get relevent information from the other components' inputs
        directive, state = self.handle_request_message(ctrl_msg)
        
        # Handle a new directive
        if self.request_recieved:
            tactic = self.tactics_algorithm.run(directive, state)
            return tactic
        else:
            return None
        
    def handle_request_message(self, ctrl_msg):
        """
        Handle a tactics request from the control component, which
        consisits of a arbitrated directive and the system state.
        """
        
        # Handle the new tactics request
        if ctrl_msg is None:
            return None, None
            
        else:
            # Split message information
            directive = ctrl_msg[0]
            state = ctrl_msg[1]
            
            # Set flag
            self.request_recieved = True
            
            return directive, state
