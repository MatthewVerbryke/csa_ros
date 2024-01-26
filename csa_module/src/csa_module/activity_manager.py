#!/usr/bin/env python3

"""
  CSA module activity manager component source code.
  
  Copyright 2021-2024 University of Cincinnati
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
        self.directive = None
        self.cur_directives = {}
        self.executing = False
        self.cur_id = -1
        self.id_count = 0
    
    def store_new_directive(self, directive):
        """
        Store a new directive and keep track of its id number
        """
        
        # Get current internal directive number
        self.cur_id = directive.id
        
        # Store directive 
        self.directive = directive

    def run_am_algorithm(self):
        """
        Run the main activity manager algorithm, and determine if it was
        successful or not.
        """
        
        response = None
        
        # Run activity manager algorithm
        output = self.am_algorithm.execute_activity(self.directive)
        directives_out = output[0]
        success = output[1]
        msg = output[2]
        
        # If failed to find directive, get failure response
        if not success:
            directives_out = None
            response = create_response_msg(self.cur_id, "", "", "failure",
                                           msg, None, "")
            
        return directives_out, response
        
    def process_new_response(self, response):
        """
        Handle response messages from controlled modules.
        """
        
        # Process response through activity manager algorithm
        mode, params = self.am_algorithm.process_response(response)        
        
        # If still waiting on other responses, continue without response
        if mode == "continue":
            response_msg = None
        
        # If successful, prepare response to control component
        elif mode == "success":
            response_msg = create_response_msg(self.cur_id, "", "", mode, "",
                                               params, "")
            self.directive = None
            self.executing = False
            
        # Otherwise get info out of response message
        elif mode == "failure":
            response_msg = create_response_msg(self.cur_id, "", "", mode,
                                               response.reject_msg, params, "")
            self.directive = None
            #TODO: issue safe directive?
        
        return response_msg
    
    def run(self, directive, response):
        """
        Run the component, based on the case most appropriate for the 
        current state of the component.
        """
        
        am_outputs = None
        ctrl_response = None
        
        # Handle getting new directive while standing-by
        if not self.executing and directive != None:
            self.store_new_directive(directive)
            am_outputs, ctrl_responses = self.run_am_algorithm()
            
            # Set 'executing' tag
            self.executing = True
            
        # Handle getting new directive while executing
        elif self.executing and directive != None:
            self.directive = None
            self.store_new_directive(directive)
            am_outputs, ctrl_responses = self.run_am_algorithm()
            
        # Handle new response on current control directive
        elif response is not None:
            ctrl_response = self.process_new_response(response)
            
        # Otherwise do nothing 
        else:
            pass
        
        return am_outputs, ctrl_response
