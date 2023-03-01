#!/usr/bin/env python3

"""
  CSA module control component source code.
  
  Copyright 2021-2022 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/csa_ros
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


import rospy

from csa_msgs.msg import Directive, Response
from csa_msgs.response import create_response_msg
from csa_msgs.directive import create_directive_msg
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
    
    def __init__(self, module_name, tactics_algorithm, latency, tolerance):

        # Initialize variables
        self.cur_id = -2
        self.directive = None
        self.tactic = None
        
        # Store input parameters
        self.latency = latency
        self.tolerance = tolerance
        
        # Flag variables
        self.executing = False
        self.continuous = False
        
        # Initialize tactics component
        self.tactics_component = TacticsComponent(tactics_algorithm) #<-- TODO: fix this
    
    def request_tactic(self, directive, state):
        """
        Request a tactic for the current state and directive, and handle 
        the outcome.
        """
        
        # Get tactic from the tactics component
        rospy.loginfo("Requesting tactic for direcitve %s...", directive.id)
        success, tactic = self.tactics_component.run(directive, state)
        
        # If successful, store tactic and new directive
        if success:
            response = None
            self.directive = directive
            self.tactic = tactic
            rospy.loginfo("Successfully got tactic")
        
        # Handle failure to find tactic
        else:
            msg = "Failed to find tactic"
            response = self.get_response_to_arbitration("failure", msg, directive)
            self.directive = None
            self.tactic = None
            rospy.loginfo("Failed to get tactic")
        
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
            arb_response = None
            if not self.executing:
                self.executing = True
        
        # Handle failure to create control directive
        # TODO: expand?
        else:
            msg = "Failed to get control directive"
            ctrl_directive = None
            arb_response = self.get_response_to_arbitration(self.directive,
                                                            "failure",
                                                            msg)
            
            # Set system to standby
            self.executing = False
            
            # Log failure to terminal/file
            rospy.loginfo(msg + " for directive %s", self.directive.id)
        
        return ctrl_directive, arb_response
        
    def process_new_response(self, response):
        """
        Handle response messages from commanded modules.
        """
        
        # Make sure this is response to current directive
        if self.cur_id != response.id:
            return None, None
        
        # Get info out of response
        if response.status == "success":
            success = True
            msg = ""
            rospy.loginfo("Directive %s execution succeded", self.directive.id)
        elif response.status == "failure":
            success = False
            msg = response.reject_msg
            rospy.loginfo("Directive %s execution failed, reason: %s", 
                self.directive.id, msg)
            # TODO: determine more about the failure?
            
        # Build response to arbitration
        arb_response = self.get_response_to_arbitration(self.directive,
                                                        response.status,
                                                        msg)
                                                        
        # Delete directive and turn off execution flag
        self.directive = None
        self.executing = False
        
        return success, arb_response
        
    def attempt_replan(self):
        """
        TODO
        """
        rospy.loginfo("Attempting to replan (currently TODO)...")
        self.executing = False
        self.directive = None
        return False, None
        
    def get_response_to_arbitration(self, mode, msg, directive):
        """
        Build a response message to the commanding module.
        """
        
        frame = None #<-- TODO: Change this?
        
        # Determine whether to use arg directive or stored directive
        if directive is None:
            id_num = self.directive.id
            source = self.directive.source
        else:
            id_num = directive.id
            source = directive.source
        
        # Create response message
        response_msg = create_response_msg(id_num,
                                           source,
                                           "",
                                           mode,
                                           msg,
                                           None,
                                           frame)
        
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
            success, arb_response = self.process_new_response(response)
            
            # Set id to unused value if successful
            if success:
                self.cur_id = -2
            
            # Ignore repeated messages for same directory
            elif success is None:
                pass
            
            # Try to replan if failure in current directive 
            else:
                got_replan, ctrl_directive = self.attempt_replan()
                if got_replan:
                    arb_response = None
                else:
                    pass
        
        # Do nothing
        else:
            pass
        
        return ctrl_directive, arb_response
