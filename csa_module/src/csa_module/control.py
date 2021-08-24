#!/usr/bin/env python

"""
  CSA module control component source code.
  
  Copyright 2021 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/gazebo_terrain
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


import rospy

from csa_msgs.msg import CSADirective, CSAResponse


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
          
    TODO: Fill out functions
    """
    
    def __init__(self, state_topic):
        
        # Get parameters
        #TODO

        # Flags
        self.directive_recieved = False
        self.tactic_recieved = False
        self.response_revieved = False
        self.pause = False
        self.switch = False
        self.failure = False
        
        # Variables
        self.directive = None
        self.tactic = None
        self.state = "standby"
        
        # Setup ROS communication objects?
        
    def run(self, arb_msg, tact_msg, am_msg):
        """
        Run the component once, reading in messages from the other
        components and updating the internal state machine
        """
        
        # Get relevent information from the other components' inputs
        self.parse_input_messages(arb_msg, tact_msg, am_msg)
        
        # Run the state machine once
        self.run_state_machine()
        
        # Create output messages
        output = self.handle_output_messages()
        
        return output
        
    def run_state_machine(self):
        """
        Run through the state machine once
        """
        
        # Wait for a new directive from arbitration
        if self.state == "standby":
            if directive_recieved:
                #ISSUE DIRECTIVE TO TACTICS
                self.state = "planning"
            else:
                pass
        
        # Wait for tactics component to return a tactic
        elif self.state == "planning":
            if self.tactic_recieved:
                self.state = "execution"
            elif not self.tactic_revieved:
                pass
            elif self.failure:
                self.state = "failure"
        
        # Execute the directive using the returned tactic
        elif self.state == "execution":
            if not response_recieved:
                self.compute_control_directive()
                #ISSUE CONTROL DIRECTIVES TO AM/LOWER MODULE
            elif self.response_recieved and self.failure:
                #ISSUE FAILURE RESPONSE TO ARB
                self.state = "failure"
            elif self.response_received and not self.failure:
                #ISSUE SUCCESS RESPONSE TO ARB
                self.state = "standby"
            elif self.pause:
                #ISSUE PAUSE DIRECTIVE TO AM/LOWER MODULE
                self.state = "pause"
            elif self.switch:
                #ISSUE SWITCH DIRECTIVE TO AM/LOWER MODULE
                self.state = "switching"
        
        # Pause the execution of the directive
        elif self.state == "paused":
            if not self.pause:
                #ISSUE UNPAUSE DIRECTIVE TO AM/LOWER MODULE
                self.state = "execution"
            elif self.pause:
                pass
            elif self.failure:
                self.state = "failure"
        
        # Respond to changed directives from the arbitration component
        elif self.state == "switching":
            #TODO
            pass
            
        # Handle a failure in the current directive
        elif self.state == "failure":
            self.handle_failure()
            
    def parse_input_messages(self, arb_msg, tact_msg, am_msg):
        """
        Read out important information from the messages sent by each
        component of the module.
        
        TODO
        """
        pass
        
    def handle_output_messages(self):
        """
        Assemble ouput messages to be sent to the other components of
        the module
        
        TODO
        """
        pass
        
    def compute_control_directive(self):
        """
        Using up-to-date system state information, compute the nessecary
        outputs for the controlled modules to execute the directive.
        
        TODO
        """
        pass
        
    def handle_failure(self):
        """
        Handle failures that can happen to the component.
        
        TODO
        """
        pass
