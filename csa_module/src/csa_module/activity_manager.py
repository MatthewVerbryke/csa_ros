#!/usr/bin/env python3

"""
  CSA module activity manager component source code.
  
  Copyright 2021-2023 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/gazebo_terrain
  Additional copyright may be held by others, as reflected in the commit
  history.
  
  TODO: Test component
"""


import rospy

from csa_msgs.msg import Directive, Response
from csa_msgs.response import create_response_msg
from csa_msgs.directive import create_directive_msg


class ActivityManagerComponent(object):
    """
    A generic activity manager (am) component object for a CSA module.
    
    Description TODO
    """
    
    def __init__(self, module_name, am_algorithm):
        
        # Store parameters
        self.name = module_name
        self.am_algorithm = am_algorithm
        self.expect_resp = am_algorithm.expect_resp
        
        # Initialize variables
        self.cur_directives = {}
        self.directives = {}
        self.executing = False
    
    def store_new_directives(self, directives):
        """
        Store new directives in local dictionary with their corrosponding
        id number.
        """
        
        for direcitve in directives:
            id_num = directive.id
            self.directives.update({id_num: directive})
    
    def run_am_algorithm(self):
        """
        Run the main activity manager algorithm, and determine if it was
        successful or not. If we got desired directives to execute, move
        them out of the storage list and into the currently executing
        list.
        """
        
        # Run activity manager algorithm
        am_outputs, success, msg = am_algorithm.run(self.cur_directives,
                                                    self.directives)
        
        # Move selected directives into currently executing list
        if success:
            for output in am_outputs:
                id_num = output.id
                self.directives.pop(id_num, None)
                self.cur_directives.update({id_num: output}
            
        return am_output, success, msg
        
    def process_new_responses(self, responses):
        """
        Handle response messages from controlled modules.
        """
        
        success = True
        msg = ""
        
        # If successful, delete completed directive from executing list
        for response in responses:
            if response.status == "success":
                self.cur_directives.pop(response.id, None)
            
            # Otherwise get info out of response message
            elif response.status == "failure":
                success = False
                msg = response.reject_msg
                break
                
        return success, msg
    
    def check_for_completion(self):
        """
        Check whether or not there are more directives to execute.
        """
        
        num_directives = len(self.cur_directives) + len(self.directives)
        if num_directives == 0:
            return True
        else:
            return False
            
    def get_response_to_control(self, mode, msg, directive):
        """
        Build a response message to the control component.
        """
        
        frame = "" #<-- TODO: Change this?
        
        # Get directive ID for failed directive, if needed.
        if directive is not None:
            id_num = directive.id
        else:
            id_num = -1
        
        # Create response message
        response_msg = create_response_msg(id_num,
                                           "",
                                           "",
                                           mode,
                                           msg,
                                           None,
                                           frame)
                                                
        return response_msg
    
    def run(self, directives, responses):
        """
        Run the component, based on the case most appropriate for the 
        current state of the component.
        """
        
        am_output = None
        ctrl_response = None
        
        # Handle getting new directive while standing-by
        if not self.executing and directives is not None:
            self.store_new_directives(directives)
            am_outputs, success, msg = self.run_am_algorithm()
            if not success:
                ctrl_responses = get_response_to_control("failure", msg,
                                                         am_outputs[0])
                                                           
                #TODO: ISSUE SAFE DIRECTIVE
            
            # Set 'executing' tag if necessary
            else:
                if self.expect_resp:
                    self.executing = True
                else:
                    pass
        
        # Handle getting new directive while executing
        elif self.executing and directives is not None :
            self.directives = {}
            self.cur_directives = {}
            self.store_new_directives(directives)
            am_outputs, success, msg = self.run_am_algorithm()
            if not success:
                ctrl_responses = get_response_to_control("failure", msg,
                                                        am_outputs[0])
                #TODO: ISSUE SAFE DIRECTIVE
        
        # Handle new response on current control directive
        elif responses is not None:
            success = self.process_new_responses(resposnses)
            if success:
                more_directives = self.check_for_completion()
                
                # Determine whether there are more directives or not
                if more_directives:
                    am_outputs, success, msg = self.run_am_algorithm()
                    if not success:
                        ctrl_responses = get_response_to_control("failure",
                                                                 msg,
                                                                 am_outputs[0])
                        #TODO: ISSUE SAFE DIRECTIVE
                else:
                    ctrl_responses = self.get_response_to_control("success", "",
                                                                  am_outputs[0])
                    self.executing = False
                    
            # Handle failed response
            else:
                ctrl_response = responses
                #TODO: ISSUE SAFE DIRECTIVE
        
        # Otherwise do nothing 
        else:
            pass
        
        return am_output, ctrl_responses
