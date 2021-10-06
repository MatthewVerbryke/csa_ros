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
    
    def __init__(self, module_name, merge_algorithm):
        
        # Get Parameters
        self.module_name = module_name
        self.merge_algorithm = merge_algorithm
        
        # Flags
        self.directive_recieved = False
        self.response_recieved = False
        self.more_directives = False
        self.issue_directive = False
        self.response_to_cmd = None
        self.failure = False
        
        # Variables
        self.directives = {}
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
        self.response_to_cmd = None
        self.new_directive = None
        self.old_directive = None
        self.response = None
        
        # Handle input messages from ctrl and the commanding module
        self.handle_new_directive(directive_msg)
        self.handle_ctrl_response(ctrl_msg)
        
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
                self.merge_directives()
                self.state = "waiting"
                
            else:
                pass
                
        # Wait for ctrl to respond with a status on the directive
        elif self.state == "waiting":
            
            if not self.directive_recieved and not self.response_recieved:
                pass
                
            elif self.directive_recieved:
                self.merge_directives()
                
            elif self.response_recieved and self.failure:
                self.state = "failure"
                
            elif self.response_recieved and len(self.directives) == 0:
                self.state = "standby"
                
            elif self.response_recieved and len(self.directives) == 1:
                self.merge_directives()

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
            # Reject new directive if we already have two in storage
            if len(self.directives) == 2:
                self.response_to_cmd = "reject"
                self.reject_msg = "Tried to append a third directive"
            else:
                # Store directive
                self.directives.update({directive.header.seq: directive})
                self.directive_recieved = True
            
            # Store new directive (even if rejected)
            self.new_directive = directive
            
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
                self.old_directive = self.directives[self.cur_id]
                self.directives.pop(self.cur_id)
        
            # Set flags
            self.response_to_cmd = "complete"
            
            # Store the response
            self.response = response
            
            # Set flag
            self.response_recieved = True
            
    def merge_directives(self):
        """
        Run the merging algorithm and determine what to do with the 
        result.
        """
        
        # Merge the current list of directives
        action, directive = self.merge_algorithm.run(self.directives)
        
        # Option 1: Setup the new directive to be issued
        if action[0] == "issue":
            self.issue_directive = True
            self.arb_directive = directive
            self.cur_id = directive.header.seq
            if self.directive_recieved:
                self.response_to_cmd = "accept"
        
        # Option 2: Accept but continue with current directive
        elif action[0] == "continue":
            self.response_to_cmd = "accept"
            
        # Option 3: Reject the directive
        elif action[0] == "reject":
            self.response_to_cmd = "reject"
            self.reject_msg = action[1]
            
    def handle_output_messages(self):
        """
        Assemble and return output messages for the component.
        """
        
        to_ctrl = None
        to_cmd = None
        
        # Issue a merged directive
        if self.issue_directive:
            to_ctrl = self.arb_directive
        
        # Respond to the commanding module
        if self.response_to_cmd == "complete" and not self.failure:
            to_cmd = create_response_msg(self.old_directive.header.seq,
                                         self.module_name,
                                         self.old_directive.source,
                                         "success",
                                         "")
        elif self.response_to_cmd == "complete" and self.failure:
            to_cmd = create_response_msg(self.directives[self.cur_id].header.seq,
                                         self.module_name,
                                         self.directives[self.cur_id].source,
                                         "failure",
                                         self.response.reject_msg)
        elif self.response_to_cmd == "accept":
            to_cmd = create_response_msg(self.new_directive.header.seq,
                                         self.module_name,
                                         self.new_directive.source,
                                         "accept",
                                         "")
        elif self.response_to_cmd == "reject":
            to_cmd = create_response_msg(self.new_directive.header.seq,
                                         self.module_name,
                                         self.new_directive.source,
                                         "failure",
                                         self.reject_msg)
                
        return [to_ctrl, to_cmd]
