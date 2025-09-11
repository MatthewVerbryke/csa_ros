#!/usr/bin/env python3

"""
  CSA module activity manager component source code.
  
  Copyright 2021-2025 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/gazebo_terrain
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


import rospy

from csa_common.am_pass_through import PassThroughActivityManager
from csa_msgs.msg import Directive, Response
from csa_msgs.response import create_response_msg


class ActivityManagerComponent(object):
    """
    A generic activity manager (am) component object for a CSA module.
    """
    
    def __init__(self, module_name, am_algorithm, prefix):
        
        # Store parameters
        self.module_name = module_name
        self.am_algorithm = am_algorithm
        self.expect_resp = am_algorithm.expect_resp
        
        # Adjust destination names if subsystem prefix exists
        if self.am_algorithm.use_prefix:
            self.am_algorithm.adjust_dest_names(prefix)
        
        # Initialize variables
        self.directive = None
        self.cur_directives = {}
        self.executing = False
        self.deadline = None
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
        
        # Store deadline (if exists)
        if directive.params.deadline == rospy.Time(0.0):
            self.deadline = None
        else:
            self.deadline = self.directive.params.deadline
        
    def run_am_algorithm(self):
        """
        Run the main activity manager algorithm, and determine if it was
        successful or not.
        """
        
        response = None
        
        # Run activity manager algorithm
        rospy.logdebug("Running activity manager")
        output = self.am_algorithm.execute_activity(self.directive)
        directives_out = output[0]
        success = output[1]
        
        # If failed to find directive, get failure response
        if not success:
            directives_out = None
            response = create_response_msg(self.cur_id, "", "", "failure",
                                           msg, None, "")
            rospy.logwarn("'{}' failed to find activities".format(
                self.module_name))
        else:
            act_msg = ""
            for key,value in directives_out.items():
                self.cur_directives.update({value.destination: value})
                act_msg += "{} -> {},".format(value.name, value.destination)
            rospy.logdebug("Executing activities {}".format(act_msg))
        
        return directives_out, response
        
    def process_new_response(self, response):
        """
        Handle response messages from controlled modules.
        """
        
        # Process response through activity manager algorithm
        mode, params = self.am_algorithm.process_response(response)        
        
        # If still waiting on other responses, continue without response
        if mode == "continue":
            self.cur_directives.pop(response.source)
            response_msg = None
        
        # If successful, prepare response to control component
        elif mode == "success":
            response_msg = create_response_msg(self.cur_id, "", "", mode, "",
                                               params, "")
            rospy.logdebug("Activities for directive {} succeeded".format(
                 self.cur_id))
            self.directive = None
            self.cur_directives = {}
            self.executing = False
            self.deadline = None
            
        # Otherwise get info out of response message
        elif mode == "failure":
            response_msg = create_response_msg(self.cur_id, "", "", mode,
                                               response.reject_msg, params, "")
            rospy.logdebug("Activities for directive {} failed: {}".format(
                 self.cur_id, response.reject_msg))
            self.directive = None
            #TODO: issue safe directive?
        
        return mode, response_msg
    
    def check_deadline(self, resp_mode=None):
        """
        Determine if current directive has run past its deadline, and if
        so, construct a failure response to control.
        
        TODO: Test
        """
        
        response_msg = None
        
        # Ignore check if no deadline is provided
        if self.deadline == None:
            past_deadline = False
        
        # Check if deadline has passed or not
        else:
            cur_time = rospy.Time.now()
            if self.deadline < cur_time:
                past_deadline = True
            elif self.deadline > cur_time:
                past_deadline = False
                
            # Unlikely but possible check for being at deadline
            elif self.deadline == cur_time:
                if resp_mode == "success" or resp_mode == "failure":
                    past_deadline = False # just handle stuff at control
                else:
                    past_deadline = True
        
        # Handle various things if deadline has passed
        if past_deadline:
            reject_msg = "Deadline ({}) reached".format(self.deadline.to_sec())
            response_msg = create_response_msg(self.cur_id, "", "", "failure",
                                               reject_msg, None, "")
            rospy.logwarn("'{}' reached deadline {} for directive {} '{}'".format(
                self.module_name,
                self.deadline.to_sec(),
                self.cur_id,
                self.directive.name
            ))
            self.directive = None
            self.executing = False
            self.deadline = None
            
        #TODO: more actions w.r.t. commanded modules?
        
        return response_msg
    
    def run(self, directive, response):
        """
        Run the component, based on the case most appropriate for the 
        current state of the component.
        """
        
        am_outputs = None
        ctrl_response = None
        
        # Handle getting new directive while standing-by
        if not self.executing and directive is not None:
            self.store_new_directive(directive)
            am_outputs, ctrl_responses = self.run_am_algorithm()
            
            # Set 'executing' tag
            if am_outputs is not None:
                self.executing = True
        
        # Handle getting new directive while executing
        elif self.executing and directive is not None:
            self.directive = None
            self.store_new_directive(directive)
            am_outputs, ctrl_responses = self.run_am_algorithm()

        # Handle new response on current control directive
        elif self.executing and response is not None:
            mode, ctrl_response = self.process_new_response(response)
            
        # Check directive deadline
        elif self.executing:
            ctrl_response = self.check_deadline()
        
        # Otherwise do nothing 
        else:
            pass

        return am_outputs, ctrl_response
    
    def reset(self):
        """
        Reset the component, including removal of all directives/
        activities and setting all variables to their original values.
        """
        
        self.directive = None
        self.cur_directives = {}
        self.executing = False
        self.deadline = None
        self.cur_id = -1
        self.id_count = 0
