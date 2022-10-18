#!/usr/bin/env python3

"""
  CSA module control component source code.
  
  Copyright 2021-2022 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/csa_ros
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
    
    def request_tactic(self, directive, state):
        """
        Request a tactic for the current state and directive, and handle 
        the outcome.
        """
        
        # Get tactic from the tactics component
        tactic, success = self.tactics_component.run(directive, state)
        
        # If successful, store tactic and new directive
        if success:
            response = None
            self.directive = directive
            self.tactic = tactic
        
        # Handle failure to find tactic
        else:
            msg = "Failed to find tactic"
            response = self.get_response_to_arbiration("failure", msg, directive)
            self.directive = None
            self.tactic = None
        
        return success, response
        
    def attempt_replan(self):
        """
        
        """
        pass
        
    def get_response_to_arbiration(self, mode, msg, directive=None):
        """
        Build a response message to the commanding module.
        """
        
        frame = None #<-- TODO: Change this?
        
        # Determine whether to use arg directive or stored directive
        if directive is None:
            id_num = self.directive.id
            source = self.directive.source
        else:
            id_num = directive.id
            source = directive.source
        
        # Create response message
        response_msg = create_response_msg(id_num, source, "", mode, msg, None,
                                           frame)
        
        return response_msg
        
    def run(self, directive, response, state):
        """
        Run the component, based on the case most appropriate for the 
        current state of the component
        """
        
        arb_response = None
        ctrl_directive = None
        
        # Check if there is new directive
        if directive is not None:
            new_directive = True
        else:
            new_directive = False
        
        # Check if there is new response
        if response is not None:
            new_response = True
        else:
            new_response = False
        
        # Handle getting new directive while standing-by
        if not self.executing and new_directive:
            got_tact, arb_response = self.request_tactic(directive, state)
            if got_tact:            
                got_dir, ctrl_directive = self.tactic.run(state)
                self.executing = True
        
        # Handle getting new directive while executing
        elif self.executing and new_directive:
            got_tact, arb_response = self.request_tactic(directive, state)
            if got_tact:
                got_dir, ctrl_directive = self.tactic.run(state)
            else:
                self.executing = False       
                
        # Handle new response on current control directive
        elif self.executing and new_response:
            if response.status == "success":
                arb_response = self.get_response_to_arbiration("success", "",
                                                               directive)
                
                # Standby if no new directive
                if not new_directive:
                    self.directive = None
                    self.executing = False
                    
            # Handle failure in current directive        
            else:
                got_replan, ctrl_directive = self.attempt_replan()
                if got_replan:
                    pass
                else:
                    msg = "Failure in current directive"
                    arb_response = self.get_response_to_arbiration("fail", msg,
                                                                   directive)
        
        # Do nothing
        else:
            pass
        
        return ctrl_directive, arb_response
