#!/usr/bin/env python3

"""
  CSA module control component source code.
  
  Copyright 2021-2022 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/gazebo_terrain
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


import rospy

from csa_msgs.msg import Directive, Response
from csa_msgs.response import create_response_msg
from csa_msgs.directive import create_directive_msg
from csa_module.tactics import TacticsComponent


class ControlComponent(object):
    """
    A generic control (ctrl) component object for a CSA module.
    
    Performs overall function of the module:
        - Recieves latest directive from arbitration
        - Consults tactics for control approach (i.e. 'tactic') to 
          achieve directive
        - Computes output directive from recieved tactic
        - Issues directives to activity manager or other controlled
          modules
        - Monitors system state infomation for the whole module
        - Reports success/failure of the merged directive to the 
          arbitration component
    """
    
    def __init__(self, module_name, tactics_algorithm):

        # Initialize variables
        self.cur_id = 1
        self.directive = None
        self.tactic = None
        
        # Flag variables
        self.executing = False
        self.continuous = False
        
        # Initialize tactics component
        self.tactics_component = TacticsComponent(tactics_algorithm) #<-- TODO: fix this
        
    def get_response_to_arbiration(self, mode):
        """
        Build a response message to the commanding module.
        """
        
        # Create a response message
        response_msg = create_response_msg(self.directive.id,
                                           "",
                                           self.directive.source,
                                           mode,
                                           "")
        
        return response_msg
        
    def run(self, directive, response, state):
        """
        Run the component, based on the case most appropriate for the 
        current state of the component
        """
        
        arb_response = None
        ctrl_directive = None
        
        # Check if we have a new directive
        if directive is not None:
            new_directive = True
        else:
            new_directive = False
        
        # Check is we have a new response
        if response is not None:
            new_response = True
        else:
            new_response = False
            
        # Handle getting a new directive while standing-by
        if not self.executing and new_directive:
            
            # Get a tactic from the tactics component
            tactic, success = self.tactics_component.run(directive, state)
            
            # Get and issue a control directive with tactic
            if success:
                self.directive = directive
                self.tactic = tactic
                ctrl_directive = self.tactic.run(state)
                self.executing = True
            else:
                pass #TODO: Handle failure to find tactic
        
        # Handle getting a new directive while executing
        elif self.executing and new_directive:
            
            # Get a tactic from the tactics component
            # #NOTE: Currently same as above, but will change
            tactic, success = self.tactics_component.run(directive, state)
            
            # Get and issue a control directive with tactic
            if success:
                self.directive = directive
                self.tactic = tactic
                ctrl_directive = self.tactic.run(state)
                # TODO: is a smoothing/transition necessary?
            else:
                pass #TODO: Handle failure to find tactic           
                
        # Handle a new response on the current control directive
        elif self.executing and new_response:
            
            # Get relevant information from response
            resp_status = response.status
            
            # Handle the response
            if resp_status == "success":
                arb_response = self.get_response_to_arbiration("success")
                if new_directive:
                    pass #TODO: change?
                else:
                    self.directive = None
                    self.executing = False
            else:
                pass #TODO: Handle a failure in the current directive
        
        # Do nothing
        #TODO: continous feeding of ctrl directives
        else:
            pass
        
        return ctrl_directive, arb_response
