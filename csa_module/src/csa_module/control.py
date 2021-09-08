#!/usr/bin/env python

"""
  CSA module control component source code.
  
  Copyright 2021 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/gazebo_terrain
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


import rospy

from csa_msgs.msg import Directive, Response
from csa_msgs.response import create_response_msg
from csa_msgs.directive import create_directive_msg


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
          
    TODO: Test member functions
    """
    
    def __init__(self):

        # Flags
        self.directive_recieved = False
        self.tactic_recieved = False
        self.response_revieved = False
        self.failure = False
        self.success = False
        
        # Variables
        self.cur_id = 0
        self.directive = None
        self.tact_directive = None
        self.tactic = None
        self.ctrl_directive = None
        self.response = None
        self.system_state = None
        self.state = "standby"

    def run(self, arb_msg, tact_msg, response_msg, state_msg):
        """
        Run the component once, reading in messages from the other
        components and updating the internal state machine
        """
        
        # Set flags and certain variables for the current loop
        self.directive_recieved = False
        self.tactic_directive = None
        self.tactic_recieved = False
        self.response_recieved = False
        self.success = False
        self.response = None
        
        # Get relevent information from the other components' inputs
        self.handle_arbitrated_directive(arb_msg)
        self.handle_tactic_return(tact_msg)
        self.handle_response(response_msg)
        
        # Store the system state
        self.system_state = state_msg
        
        # Run the state machine once
        self.run_state_machine()
        
        # Create output messages
        output = self.handle_output_messages()
        
        return output
        
    def run_state_machine(self):
        """
        Run through the state machine once
        """
        
        # Wait for a new directive from arbitration
        if self.state == "standby":
            if self.directive_recieved:
                self.tactic_directive = [self.directive, self.system_state]
                self.state = "planning"
            else:
                pass
        
        # Wait for tactics component to return a tactic
        elif self.state == "planning":
            if self.tactic_recieved and not self.failure:
                self.state = "execution"
            elif self.tactic_recieved and self.failure:
                self.state = "failure"
            elif not self.tactic_recieved:
                pass
                
        
        # Execute the directive using the returned tactic
        elif self.state == "execution":
            if self.directive_recieved: # TODO: improve this?
                self.compute_control_directive()
                self.tactic_directive = [self.directive, self.system_state]
                self.state = "planning"
            elif not self.response_recieved:
                self.compute_control_directive()
            elif self.response_recieved and self.failure:
                self.state = "failure"
            elif self.response_recieved and self.success:
                self.state = "standby"
            
        # Handle a failure in the current directive
        elif self.state == "failure":
            self.handle_failure()
            
    def handle_arbitrated_directive(self, arb_directive):
        """
        Handle directive inputs from commanding module(s).
        """
        
        # Handle the new directive
        if arb_directive is None:
            pass
        
        else:
            #TODO: handle switching between directives during execution
            
            # Store new directive
            self.directive = arb_directive
            
            # Set flag
            self.directive_recieved = True
        
    def handle_tactic_return(self, tactic_msg):
        """
        Handle a return message from the tactics component of the 
        module.
        """
        
        # Handle the new tactics message
        if tactic_msg is None:
            pass
            
        else:
            # See if tactics failed
            if tactic_msg.status == "failure":
                self.failure = True
            
            # Store the received tactic
            else:
                self.tactic = tactic_msg
                
            # Set flag
            self.tactic_recieved = True
        
    def handle_response(self, response):
        """
        Handle a response from the activity manager component of the 
        module or an external commanded module.
        """
        
        # Handle the new response
        if response is None:
            pass
        
        else:
            # Check if the commanded directive succeded or failed
            if response.status == "failure":
                self.failure = True
            elif response.status == "success":
                self.success = True
                #TODO: do more here?
            
            # Store the response
            self.response = response
            
            # Set flag
            self.response_recieved = True
    
    def handle_output_messages(self):
        """
        Assemble ouput messages to be sent to the other components of
        the module
        """
        
        # Handle the control directive outputs
        if self.ctrl_directive is None:
            to_cmd = None
        elif self.failure or self.success:
            to_cmd = None
        else:
            to_cmd = self.ctrl_directive
            
        # Handle a request to the tactic component for a tactic
        if self.tactic_directive is None:
            to_tact = None
        else:
            to_tact = self.tactic_directive
            
        # Handle issuance of a response to the arbitration component
        if self.response_recieved:
            if self.failure:
                fail_msg = self.response.reject_msg
                to_arb = create_response_msg(self.cur_id, "failure", fail_msg)
            else:
                to_arb = create_response_msg(self.cur_id, "success", "")
        else:
            to_arb = None
            
        return [to_arb, to_tact, to_cmd]
        
    def compute_control_directive(self):
        """
        Using up-to-date system state information, compute the nessecary
        outputs for the controlled modules to execute the directive.
        
        TODO: More logic to catch failures
        """
        
        # Run the control tactic
        self.ctrl_directive = self.tactic.run(self.system_state)
        
    def handle_failure(self):
        """
        Handle failures that can happen to the component.
        
        TODO: Impliment this functionality
        """
        self.state = "standby"
        self.failure = False
        self.ctrl_directive = None
