#!/usr/bin/env python3

"""
  CSA module activity manager component source code.
  
  Copyright 2021-2023 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/gazebo_terrain
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


import rospy

from csa_msgs.msg import Directive, Response
from csa_msgs.response import create_response_msg
from csa_msgs.directive import create_directive_msg


class ActivityManagerComponent(object):
    """
    A generic activity manager (am) component object for a CSA module.
    """
    
    def __init__(self, module_name, am_algorithm):
        
        # Store parameters
        self.name = module_name
        self.am_algorithm = am_algorithm
        self.expect_resp = am_algorithm.expect_resp
        
        # Initialize variables
        self.cur_directives = {}
        self.directives = {}
        self.responses = []
        self.executing = False
        self.cur_id = -1
        self.id_count = 1
    
    def store_new_directives(self, directives):
        """
        Store new directives in local dictionary with their corrosponding
        id number.
        """
        
        # Get current internal directive number
        self.cur_id = directives[0].id
        
        # Store directive with new ids
        for direct in directives:
            direct.id = self.id_count
            self.directives.update({self.id_count: direct})
            
            # Increment id counter
            self.id_count += 1

    def run_am_algorithm(self):
        """
        Run the main activity manager algorithm, and determine if it was
        successful or not. If we got directives to execute, move
        them out of the storage list and into the currently executing
        list.
        """
        
        response = None
        
        # Run activity manager algorithm
        directives_out, success, msg = self.am_algorithm.run(self.directives)
        
        # Move selected directives into currently executing list
        if success:
            for key,value in directives_out.items():
                self.directives.pop(key, None)
                self.cur_directives.update({key: value})
        
        # If failed to find directive get failure response
        else:
            directives_out = None
            response = create_response_msg(self.cur_id, "", "", "failure",
                                           msg, None, "")
            
        return directives_out, response
        
    def process_new_responses(self, response):
        """
        Handle response messages from controlled modules.
        """
        
        # If successful, delete completed directive from executing list
        if response.status == "success":
            success = True
            self.cur_directives.pop(response.id, None)
            self.response = None
            
        # Otherwise get info out of response message
        elif response.status == "failure":
            success = False
            self.response = response
                
        return success
        
    def get_response_to_control(self, mode):
        """
        Build a response message to the control component.
        """
        
        # If sequence is complete send responses
        if mode == "success":
            if len(self.directives) == 0:
                response_msg = create_response_msg(self.cur_id, "", "", mode,
                                                    "", None, "")
            
            # Otherwise continue
            else:
                response_msg = None
            
        # Pass through failure message for direcive which failed
        else:
            response_msg = create_response_msg(self.cur_id, "", "", mode,
                                               self.response.reject_msg, None,
                                               "")
            self.clear_directives()
                                                
        return response_msg
    
    def clear_directives(self):
        """
        Clear all stored direcitve information.
        """
        
        self.directives = {}
        self.cur_directives = {}
        self.response = None
    
    def run(self, directives, response):
        """
        Run the component, based on the case most appropriate for the 
        current state of the component.
        """
        
        am_outputs = None
        ctrl_responses = None
        
        # Handle getting new directive while standing-by
        if not self.executing and directives != [None]:
            self.store_new_directives(directives)
            am_outputs, ctrl_responses = self.run_am_algorithm()
            
            # Set 'executing' tag
            self.executing = True
        
        # Handle getting new directive while executing
        elif self.executing and directives != [None]:
            self.clear_directives()
            self.store_new_directives(directives)
            am_outputs, ctrl_responses = self.run_am_algorithm()
        
        # Handle new response on current control directive
        elif response is not None:
            success = self.process_new_responses(response)
            
            # If success prepare response
            if success:
                ctrl_responses = self.get_response_to_control("success")
                
                # Rerun AM if still directives to run
                if ctrl_responses is None:
                    am_outputs, ctrl_responses = self.run_am_algorithm()
                
                # If no new directives stop executing
                else:
                    self.executing = False
            
            # Pass through failure response to control
            else:
                ctrl_responses = self.get_response_to_control("failure")
                #TODO: ISSUE SAFE DIRECTIVE
        
        # Otherwise do nothing 
        else:
            pass
        
        return am_outputs, ctrl_responses
