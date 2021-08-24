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

from csa_msgs.msg import CSADirective, CSAResponse


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
        
    TODO: Fill out functions
    """
    
    def __init__(self, command_topic, response_topic, merge_algorithm):
        """
        
        """
        
        # Get Parameters
        self.merge_algorithm = merge_algorithm
        
        # Flags
        self.directive_recieved = False
        self.response_recieved = False
        self.new_priority = False
        self.more_directives = False
        
        # Variables
        self.directives = []
        self.arbitrated_directive = None
        self.state = "standby"
        
        # Setup ROS communication objects?
        
    def run(self, ctrnotl_msg):
        """
        Run the component once, reading in messages from the other
        components and updating the internal state machine.
        """
        
        # Get relevent information from the ctrl component inputs
        self.parse_ctrl_message(ctrl_msg)
        
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
                self.merge_directives(False, True)
                self.state = "waiting"
            else:
                pass
                
        # Wait for ctrl to respond with a status on the directive
        elif self.state == "waiting":
            if not self.directive_recieved and not self.response recieved:
                pass
            elif self.directive_recieved and self.new_priority:
                self.merge_directives(False, False)
                self.state = "switching"
            elif self.directive_recieved and not self.new_priority:
                self.merge_directives(False, False)
            elif self.response_recieved and self.failure:
                #SIGNAL FAILURE OF DIRECTIVE
                self.merge_directives(True, False)
            elif self.response_recieved and not self.more_directives:
                #SIGNAL COMPLETION OF DIRECTIVE
                self.state = "standby"
            elif self.response_recieved and self.more_directives:
                #SIGNAL COMPLEITION OF DIRECTIVE
                self.merge_directives(True, True)
              
        # Respond to the need to switch arbitrated directives
        elif self.state == "switching":
            #TODO
            pass

        elif self.state == "failure":
            if self.continue_next_directive:
                #ISSUE ARBITRATED DIRECTIVE TO CTRL COMPONENT
                self.state = "waiting"
            else:
                self.state = "standby"
                
    def parse_ctrl_message(self, ctrl_msg):
        """
        
        """
        pass
        
    def handle_output_messages(self):
        """
        
        """
        pass
        
    def merge_directives(self, del_current, issue_directive):
        """
        
        """
        pass
        
    
