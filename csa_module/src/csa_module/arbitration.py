#!/usr/bin/env python3

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
        
        # Initialize other parameters
        self.cur_id = -1
        self.cur_directive = None
        self.max = 2
        self.failure = False
        
        # Initialize storage variables
        self.directives = {}
       
    def process_new_directive(self, directive):
        """
        Handle directive inputs from commanding module(s).
        """
        
        # Reject new directives over the capacity limit
        if len(self.directives) == self.max:
            is_okay = False
            msg = "Tried to append more than {} directives".format(self.max)
            
        # TODO: Add more basic checks here
        #elif
        
        # Store the new directive
        else:
            self.directives.update({directive.header.seq: directive})
            is_okay = True
            msg = ""
        
        return is_okay, msg
       
    def get_response_to_commander(self, directive, msg_type, msg):
        """
        Build a response message to the commanding module.
        """
        
        # Create a response message
        response_msg = create_response_msg(directive.id,
                                           self.module_name,
                                           directive.source,
                                           msg_type,
                                           msg)
        
        return response_msg
        
    def merge_directives(self):
        """
        Merge the directives stored in the module to get an arbitrated
        directive to send to control.
        """
        
        # Merge the directives to get an arbitrated directive
        arb_directive = self.merge_algorithm.run(self.directives)
        
        # Check whether to issue the directive or not
        if arb_directive.id!= self.cur_id:
            self.cur_id = arb_directive.header.seq
            self.cur_directive = arb_directive
            return arb_directive
        else:
            return None
        
    def run(self, directive, response):
        """
        Run the component, based on the case most appropriate for the 
        current state of the component
        """
        
        arb_directive = None
        cmdr_msg = None
        
        # Process new directive
        if directive is not None:
            is_okay, msg = self.process_new_directive(directive)
            
            # Respond to commanding module(s)
            if is_okay:
                msg_type = "accept"
            else:
                msg_type = "reject"
            cmdr_msg = self.get_response_to_commander(directive, msg_type, msg)
            
            # Arbitrate over directives
            if is_okay:
                arb_directive = self.merge_directives()
       
        # Process new response
        elif response is not None:
            if response.status == "failure":
                self.failure = True
                msg_type = response.status
                msg = response.reject_msg
            elif response.status == "success":
                msg_type = response.status
                msg = ""
            
            # Respond to commanding module(s)
            cmdr_msg = self.get_response_to_commander(self.cur_directive, msg_type, msg)
            
            # Cleanup directive
            if not self.failure:
                self.directives.pop(response.id)
                
                # Arbitrate over directives
                if len(self.directives) != 0:
                    arb_directive = self.merge_directives()
                
            #TODO: need to handle 'if failure:'
            else:
                print("TODO")

        return arb_directive, cmdr_msg
