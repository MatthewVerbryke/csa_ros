#!/usr/bin/env python3

"""
  CSA module activity manager component source code.
  
  Copyright 2021-2023 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/gazebo_terrain
  Additional copyright may be held by others, as reflected in the commit
  history.
  
  TODO: Test rework
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
        self.directives = {}
        self.cur_directives = {}
        self.executing = False
        self.cur_id = -1
        self.id_count = 0
    
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
        
        # If failed to find directive, get failure response
        else:
            directives_out = None
            response = create_response_msg(self.cur_id, "", "", "failure",
                                           msg, None, "")
            
        return directives_out, response
        
    def process_new_responses(self, response):
        """
        Handle response messages from controlled modules.
        """
        
        # Process response through activity manager algorithm
        mode, params = self.am_algorithm.process_response(response)        
        
        # If still waiting on other responses, continue without response
        if mode == "contintue":
            self.cur_directives.pop(response.id, None)
            response_msg = None
        
        # If successful, prepare response to control component
        elif mode == "success":
            self.cur_directives.pop(response.id, None)
            response_msg = create_response_msg(self.cur_id, "", "", mode,
                                               "", params, "")
            
            # Reset execution flag
            self.executing = False
            
        # Otherwise get info out of response message
        elif mode == "failure":
            response_msg = create_response_msg(self.cur_id, "", "", mode,
                                               self.response.reject_msg, params,
                                               "")
            
            # Clear directives
            self.clear_directives()
                
        return response_msg
        
    def clear_directives(self):
        """
        Clear all stored direcitve information.
        """
        
        self.directives = {}
        self.cur_directives = {}
    
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
            ctrl_response = self.process_new_responses(response)
        
        # Otherwise do nothing 
        else:
            pass
        
        return am_outputs, ctrl_responses
