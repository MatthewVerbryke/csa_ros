#!/usr/bin/env python3

"""
  CSA module control component source code.
  
  Copyright 2021-2024 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/csa_ros
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


import rospy

from csa_msgs.msg import Directive, Response
from csa_msgs.response import create_response_msg
from csa_module.tactics import TacticsComponent


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
    """
    
    def __init__(self, module_name, tactics_algorithm, rate, latency, model):

        # Initialize variables
        self.cur_id = -2
        self.directive = None
        self.tactic = None
        
        # Store input parameters
        self.module_name = module_name
        self.rate = rate
        self.latency = latency
        self.model = model
        
        # Flag variables
        self.executing = False
        
        # Initialize tactics component
        # TODO: fix this?
        self.tactics_component = TacticsComponent(module_name, 
                                                  tactics_algorithm,
                                                  rate,
                                                  latency)
    
    def request_tactic(self, directive, state):
        """
        Request a tactic for the current state and directive, and handle 
        the outcome.
        """
        
        # Get tactic from the tactics component
        rospy.loginfo("'{}': Requesting tactic for direcitve {}...".format(
            self.module_name, directive.id))
        success, tactic = self.tactics_component.run(directive, state,
                                                     self.model)
        
        # If successful, store tactic and new directive
        if success:
            response = None
            self.directive = directive
            self.tactic = tactic
            rospy.loginfo("'{}': Successfully found tactic, exectuing...".format(
                self.module_name))
        
        # Handle failure to find tactic
        else:
            msg = "Failed to find tactic"
            response = self.get_response_to_arbitration(directive, "failure",
                                                        msg)
            self.directive = None
            self.tactic = None
            rospy.logwarn("'{}': {}".format(self.module_name, msg))
        
        return success, response
        
    def create_control_directive(self, state):
        """
        Given the current system state, create a control directive to 
        issue to a commanded module.
        """
        
        # Create control directive
        got_dir, ctrl_directive = self.tactic.run(state)
        
        # Handle successfully finding control directive
        if got_dir:
            if not self.tactic.resp_output:
                arb_response = None
                if not self.executing:
                    self.executing = True
                    
                # If tactic indicates completion, get success response
                if self.tactic.completed:
                    rospy.loginfo("'{}': Directive {} execution complete".format(
                        self.module_name, self.directive.id))
                    arb_response = self.get_response_to_arbitration(None,
                                                                    "success",
                                                                    "")
            
            # Build response if not issuing directives for tactic
            # TODO: Test
            else:
                if ctrl_directive is None:
                    arb_response = None
                    ctrl_directive = None
                else:
                    params = ctrl_directive.params
                    ctrl_directive = None
                    arb_response = self.get_response_to_arbitration(None,
                                                                    "success",
                                                                    "", params)
        
        # Handle failure to create control directive
        # TODO: expand?
        else:
            msg = "Failed to get control directive"
            ctrl_directive = None
            arb_response = self.get_response_to_arbitration(None, "failure",
                                                            msg)
            
            # Set system to standby
            self.executing = False
            
            # Log failure to terminal/file
            rospy.logwarn("'{}': {}".format(self.module_name, msg))
        
        return ctrl_directive, arb_response
    
    def process_new_response(self, response):
        """
        Handle response messages from commanded modules.
        """
        
        # Make sure this is response to current directive
        if self.cur_id != response.id:
            return None, None, None
            
        # Determine reaction to response
        if self.tactic.evals_resp:
            mode, params = self.tactic.evaluate_response(response)
        else:
            mode = response.status
            params = None
        
        # Determine output reaction based on reaction
        if mode == "continue":
            success = True
            rerun = True
            arb_response = None
        elif mode == "success":
            success = True
            rerun = False
            rospy.loginfo("'{}': Directive {} execution succeded".format(
                self.module_name, self.directive.id))
            arb_response = self.get_response_to_arbitration(self.directive,
                                                            "success", "",
                                                            params)
        elif mode == "failure":
            success = False
            rerun = False
            rospy.logwarn("'{}': Directive {} execution failed, {}".format(
                self.module_name, self.directive.id, response.reject_msg))
            arb_response = self.get_response_to_arbitration(self.directive,
                                                            "failure",
                                                            response.reject_msg)
            # TODO: determine more about the failure?
        
        return success, rerun, arb_response
        
    def attempt_replan(self):
        """
        TODO
        """
        rospy.loginfo("'{}': Attempting to replan (currently TODO)...".format(
            self.module_name))
        self.executing = False
        self.directive = None
        return False, None
        
    def get_response_to_arbitration(self, directive, mode, msg, params=None):
        """
        Build a response message to the commanding module.
        """
        
        frame = "" #<-- TODO: Change this?
        
        # Determine whether to use arg directive or stored directive
        if directive is None:
            id_num = self.directive.id
            source = self.directive.source
        else:
            id_num = directive.id
            source = directive.source
        
        # Create response message
        response_msg = create_response_msg(id_num, source, "", mode, msg,
                                           params, frame)
        
        return response_msg
        
    def run(self, directive, response, state):
        """
        Run the component, based on the case most appropriate for the 
        current state of the component
        """
        
        arb_response = None
        ctrl_directive = None
        
        # Handle getting new directive while standing-by
        if not self.executing and directive is not None:
            got_tact, arb_response = self.request_tactic(directive, state)
            if got_tact:
                self.cur_id = directive.id
                ctrl_directive, arb_response = self.create_control_directive(
                    state)            
            else:
                pass
        
        # Handle getting new directive while executing
        elif self.executing and directive is not None:
            got_tact, arb_response = self.request_tactic(directive, state)
            if got_tact:
                self.cur_id = directive.id
                ctrl_directive, arb_response = self.create_control_directive(
                    state)
            else:
                self.executing = False       
        
        # Handle new response on current control directive
        elif self.executing and response is not None:
            success, rerun, arb_response = self.process_new_response(response)
            
            # Ignore repeated messages for same directive
            if success is None:
                pass
                
            # If successful, set id to unused value
            elif success and not rerun:
                self.cur_id = -2
                self.directive = None
                self.executing = False
            
            # Rerun tactic if need to continue
            elif success and rerun:
                ctrl_directive, arb_response = self.create_control_directive(
                    state)
            
            # Try to replan if failure in current directive 
            else:
                got_replan, ctrl_directive = self.attempt_replan()
                if got_replan:
                    arb_response = None
                else:
                    self.directive = None
                    self.executing = False
        
        # Continue or do nothing
        else:
            if self.tactic.continuous:
                ctrl_directive, arb_response = self.create_control_directive(
                    state)
            else:
                pass
        
        return ctrl_directive, arb_response
