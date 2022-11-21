#!/usr/bin/env python3

"""
  CSA module arbitration component source code.
  
  Copyright 2021-2022 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/csa_ros
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
    
    def __init__(self, module_name, merge_algorithm, default_directive):
        
        # Get Parameters
        self.module_name = module_name
        self.merge_algorithm = merge_algorithm
        
        # Setup default directive
        self.default_directive = default_directive
        self.default_directive.id = -1
        self.cur_id = -1
        
        # Initialize other parameters
        self.cur_directive = None
        self.max = 2
        self.standby = True
                
        # Initialize storage variables
        self.directives = {}
        self.hold_over = None
       
    def process_new_directive(self, directive):
        """
        Handle directive inputs from commanding module(s).
        """
        
        # Ignore if we got the directive already
        if directive.id in self.directives.keys():
            is_okay = True
            msg = "ignore"
        
        # Reject new directives over the capacity limit
        elif len(self.directives) == self.max:
            is_okay = False
            msg = "Tried to append more than {} directives".format(self.max)
        
        # TODO: Add more basic checks here
        #elif
        
        # Store the new directive
        else:
            self.directives.update({directive.id: directive})
            is_okay = True
            msg = ""
            rospy.loginfo("Accepted directive %s from %s", directive.id,
                directive.source)
        
        # Log failures with reasoning
        if is_okay == False:
            rospy.loginfo("Rejected directive %s from %s, reason: #s", 
                direcitve.id, directive.source, msg)
        
        return is_okay, msg
        
    def process_new_response(self, response):
        """
        Handle response messages from the module's control component.
        """
        
        # Get info out of response
        if response.status == "success":
            success = True
            msg = ""
        elif response.status == "failure":
            success = False
            msg = response.reject_msg
            # TODO: determine more about the failure?
        
        return success, msg
    
    def get_response_to_commander(self, directive, msg_type, msg):
        """
        Build a response message to the commanding module.
        """
        
        # Don't make a message if we are told to ignore
        if msg == "ignore":
            response_msg = None
        
        # Create a response message    
        else:
            response_msg = create_response_msg(directive.id,
                                               self.module_name,
                                               directive.source,
                                               msg_type,
                                               msg,
                                               None,
                                               None)
        
        return response_msg
        
    def issue_default_directive(self):
        """
        Return the default directive, while also setting all relevant
        variables.
        """
        
        # Make sure we are in standby mode
        self.standby = True
        
        # If already issued don't issue it again
        if self.cur_directive is not None:
            return None
            
        # Store info from default directive
        else:
            self.cur_directive = self.default_directive
            self.cur_id = -1
            rospy.loginfo("Issuing default directive...")
        
            return self.default_directive
        
    def merge_directives(self):
        """
        Merge the directives stored in the module to get an arbitrated
        directive to send to control.
        """
        
        # Merge the directives to get an arbitrated directive
        rospy.loginfo("Merging directives...")
        arb_directive = self.merge_algorithm.run(self.directives)
        
        # Check whether to issue the directive or not
        if arb_directive.id!= self.cur_id:
            self.cur_id = arb_directive.id
            self.cur_directive = arb_directive
            rospy.loginfo("Arbitration result: switching to directive %s",
                self.cur_id)
        else:
            switch = False
            arb_directive = None
            rospy.loginfo("Arbitration result: continue")
        
        return arb_directive
    
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
            
            # Get response to commanding module(s)
            if is_okay:
                msg_type = "accept"
            else:
                msg_type = "reject"
            cmdr_msg = self.get_response_to_commander(directive, msg_type, msg)
            
            # Arbitrate over directives
            if is_okay:
                if msg != "ignore":
                    arb_directive = self.merge_directives()
                    self.standby = False
                    
                    # Delete hold-over if it exists
                    self.hold_over = None
        
        # Handle "held over" directves if we didn't get new one
        elif self.hold_over is not None:
            arb_directive = self.hold_over
            self.standby = False
            self.hold_over = None
        
        # Process new response
        elif response is not None:
            success, msg = self.process_new_response(response)
            
            # Get response to commanding module(s)
            if success:
                msg_type = "success"
            else:
                msg_type = "failure"
            cmdr_msg = self.get_response_to_commander(self.cur_directive,
                                                      msg_type, msg)
            
            # Cleanup the completed/failed directive
            self.directives.pop(response.id)
            self.cur_directive = None
            
            # Arbitrate over remaining directives or issue default
            if len(self.directives) != 0:
                self.hold_over = self.merge_directives()
            else:
                self.hold_over = self.issue_default_directive()
            
        # Determine what to do with no response or new directive
        else:
            if self.standby:
                arb_directive = self.issue_default_directive()
            else:
                pass
        
        return arb_directive, cmdr_msg
