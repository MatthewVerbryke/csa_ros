#!/usr/bin/env python

"""
  CSA module arbitration component source code.
  
  Copyright 2021-2022 University of Cincinnati
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
        self.more_directives = False
        self.issue_directive = False
        self.failure = False
        
        # Variables
        self.directives = {}
        self.arb_directive = None
        self.cur_id = 0
        self.response = None
        self.response_to_cmd = None
    
    def handle_new_directive(self, directive):
        """
        Handle directive inputs from commanding module(s).
        """
        
        # Set flags and certain variables for the current function run
        self.directive_recieved = False
        self.issue_directive = False
        self.response_to_cmd = None
        self.new_directive = None 
        
        # Handle the new directive
        if directive is None:
            pass
            
        else:            
            # Reject new directive if we already have two in storage
            if len(self.directives) == 2:
                self.response_to_cmd = "reject"
                self.reject_msg = "Tried to append a third directive"
            
            # Add the new directive to the list and merge the new list
            else:
                self.directives.update({directive.header.seq: directive})
                self.directive_recieved = True
                self.merge_directives()
            
            # Store new directive (even if rejected)
            self.new_directive = directive
                
        # Create output message to CTRL and the commanding module
        output = self.create_output_messages()
        
        return output
    
    def handle_ctrl_response(self, response_msg):
        """
        Handle a response from the control component of the module.
        """
        
        # Set flags and certain variables for the current loop
        self.response_to_cmd = None
        self.old_directive = None
        self.response = None
        
        # Handle the new response
        if response_msg is None:
            pass
        
        else:
            # Check for a failed directive
            if response_msg.status == "failure":
                self.failure = True
                
            # If success remerge the directives list
            elif response_msg.status == "success":
                self.old_directive = self.directives[self.cur_id]
                self.directives.pop(self.cur_id)
                self.merge_directives()
        
            # Set flags
            self.response_to_cmd = "complete"
            
            # Store the response
            self.response = response_msg
            
            # Create output message to CTRL and commanding module
            output = self.create_output_messages()
            
            return output
            
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
            
    def create_output_messages(self):
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
