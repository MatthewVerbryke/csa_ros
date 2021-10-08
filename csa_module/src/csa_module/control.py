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
        self.issue_tactic_request = False
        self.tactic_recieved = False
        self.issue_ctrl_directive = False
        self.response_revieved = False
        self.failure = False
        
        # Variables
        self.cur_id = 0
        self.directive = None
        self.tactics_request = None
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
        self.issue_tactic_request = False
        self.tactic_recieved = False
        self.issue_ctrl_directive = False
        self.response_recieved = False
        self.tactics_request = None
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
                self.issue_tactic_request = True
                self.tactics_request = [self.directive, self.system_state]
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
                self.issue_tactic_request = True
                self.tactics_request = [self.directive, self.system_state]
                self.state = "planning"
                
            elif not self.response_recieved:
                self.compute_control_directive()
                self.issue_ctrl_directive = True # <--TODO: change this?
                
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
            # TODO: fix this section
            # See if tactics failed
            #if tactic_msg.status == "failure":
                #self.failure = True
            
            # Store the received tactic
            #else:
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
        
        to_arb = None
        to_tact = None
        to_cmd = None
        
        # Issue a control directive
        if self.issue_ctrl_directive:
            to_cmd = self.ctrl_directive
            
        # Request a tactic from the tactic component
        if self.issue_tactic_request:
            to_tact = self.tactics_request
            
        # Respond to the arbitration component
        if self.response_recieved and self.failure:
                fail_msg = self.response.reject_msg
                to_arb = create_response_msg(self.cur_id, "failure", fail_msg)
        elif self.response_recieved and not self.failure:
                to_arb = create_response_msg(self.cur_id, "success", "")
            
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
