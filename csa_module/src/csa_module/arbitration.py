#!/usr/bin/env python

"""
  CSA module arbitration component source code.
  
  Copyright 2021 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/gazebo_terrain
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


import rospy

from csa_msgs.msg import Directive, Response
from csa_msgs.response import create_response_msg


class ArbitrationComponent(object):
    """
    A generic arbitration (arb) component object for a CSA module.
    
    Manages the overall behavior of the module: 
        - Recieve directives from commanding module
        - Process the recieved directives one at a time and merge them 
          them every time a new one is recieved, to create an arbitrated
          directive
        - Issue the arbitrated directive to control component
        - Recieve and analyze reponse from control element
        - Report status back to commanding module
    """
    
    def __init__(self, merge_algorithm):
        
        # Get Parameters
        self.merge_algorithm = merge_algorithm
        
        # Flags
        self.directive_recieved = False
        self.response_recieved = False
        self.more_directives = False
        self.issue_directive = False
        self.failure = False
        
        # Variables
        self.directives = []
        self.cur_id = 0
        self.response = None
        self.merged_directive = None
        self.current_priority = 0
        self.state = "standby"
        
    def run(self, directive_msg, ctrl_msg):
        """
        Run the component once, reading in messages from the other
        components and updating the internal state machine.
        """
        
        # Set flags and certain variables for the current loop
        self.directive_recieved = False
        self.response_recieved = False
        self.issue_directive = False
        self.response = None
        
        # Handle input messages from ctrl and the commanding module
        self.handle_new_directive(directive_msg)
        self.handle_ctrl_response(ctrl_msg)
        
        # Check how many directives are left
        self.more_directives = (len(self.directives) > 1)
        
        # Run the state machine once
        self.run_state_machine()

        # Create/Publish output messages
        output = self.handle_output_messages()
        
        return output
        
    def run_state_machine(self):
        """
        Run through the state machine once.
        """

        # Wait for a new directive from the commanding module
        if self.state == "standby":
            
            if self.directive_recieved:
                self.merged_directive = self.merge_algorithm.run(self.directives)
                self.issue_directive = True
                self.state = "waiting"
            else:
                pass
                
        # Wait for ctrl to respond with a status on the directive
        elif self.state == "waiting":
            
            if not self.directive_recieved and not self.response_recieved:
                pass
            elif self.directive_recieved:
                self.merged_directive = self.merge_algorithm.run(self.directives)
                self.issue_directive = True
                # TODO: decide whether to append new directive or
                #       replace current one
            elif self.response_recieved and self.failure:
                self.state = "failure"
            elif self.response_recieved and not self.more_directives:
                self.directives.pop(0)# TODO: replace with ID based approach
                self.state = "standby"
            elif self.response_recieved and self.more_directives:
                self.directives.pop(0)# TODO: replace with ID based approach
                self.merged_directive = self.merge_algorithm.run(self.directives)
                self.issue_directive = True

        # Coordinate with commanding module on failures
        elif self.state == "failure":
            self.state = "standby"
            #TODO: determine if next directive can be substituted
    
    def handle_new_directive(self, directive):
        """
        Handle directive inputs from commanding module(s).
        """
        
        # Handle the new directive
        if directive is None:
            pass
            
        else:            
            # TODO: Look for problems and reject bad directives
            
            # Store directive
            self.directives.append(directive)
            
            # Set flag
            self.directive_recieved = True
            
    def handle_ctrl_response(self, response):
        """
        Handle a response from the control component of the module.
        """
        
        # Handle the new response
        if response is None:
            pass
        
        else:
            # Check if the commanded directive succeded or failed
            if response.status == "failure":
                self.failure = True
            elif response.status == "success":
                pass
                #TODO: do more here?
            
            # Store the response
            self.response = response
            
            # Set flag
            self.response_recieved = True
                
    def handle_output_messages(self):
        """
        Assemble and return output messages for the component.
        """
        
        # Handle issuance of merged directive
        if self.issue_directive:
            to_ctrl = self.merged_directive
        else:
            to_ctrl = None
            
        # Handle issuance of a response to the commanding module(s)
        if self.response_recieved:
            if self.failure:
                fail_msg = self.response.reject_msg
                to_cmd = create_response_msg(self.cur_id, "failure", fail_msg)
            else:
                to_cmd = create_response_msg(self.cur_id, "success", "")
        else:
            to_cmd = None
                
        return [to_ctrl, to_cmd]
