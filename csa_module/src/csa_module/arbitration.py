#!/usr/bin/env python3

"""
  CSA module arbitration component source code.
  
  Copyright 2021-2025 University of Cincinnati
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
    
    def __init__(self, module_name, merge_algorithm, max_directives, 
                 default_params=None):
        
        # Store Parameters
        self.module_name = module_name
        self.merge_algorithm = merge_algorithm
        self.max = max_directives
        
        # Fill out common details of default directive
        self.default_directive = Directive()
        self.default_directive.source = "self"
        self.default_directive.id = -1
        self.default_directive.response_time = 1.0
        self.default_directive.priority = -1 # FIXME?
        
        # Add specific default directive parameters
        if default_params is not None:
            self.default_directive.name = default_params["name"]
            self.default_directive.params = default_params["params"]
        else:
            self.default_directive.name = "standby"
            
            # Add standby tactic to allowed directives list
            self.merge_algorithm.allowed_dirs.append("standby")
        
        # Initialize other parameters
        self.cur_directive = None
        self.standby = True
        
        # Initialize storage variables
        self.directives = {}
        self.hold_over = None
        
        # Initialize other variables
        self.cur_id = -1
        self.dir_key = ""
        self.id_count = 1
        self.ignore = False
       
    def process_new_directive(self, directive):
        """
        Handle directive inputs from commanding module(s).
        """
        
        # Ignore if we got the directive already
        dir_key = directive.source + "_" + str(directive.id)
        if dir_key in self.directives.keys():
            is_okay = True
            self.ignore = True
            msg = ""
        
        # Reject new directives over the capacity limit
        elif len(self.directives) == self.max:
            is_okay = False
            msg = "Tried to append more than {} directives".format(self.max)
        
        # Check if the directive action is in the allowed list
        elif directive.name not in self.merge_algorithm.allowed_dirs:
            is_okay = False
            msg = "Directive action '{}' not allowed".format(directive.name)
        
        # Reject directive if it somehow got sent to wrong destination
        elif directive.destination != self.module_name:
            is_okay = False
            msg = "Directive destination '{}' is not this module".format(
                directive.destination)
            
        # TODO: Add more basic checks here
        
        # Store the new directive
        else:
            self.directives.update({dir_key: directive})
            is_okay = True
            rospy.logdebug("Accepted directive {} '{}' from {}".format(
                directive.id, directive.name, directive.source))
        
        # Log failures with reasoning
        if is_okay == False:
            rospy.logwarn("'{}' rejected directive {} '{}' from {}: {}".format( 
                self.module_name, directive.id, directive.name, directive.source,
                msg))
            msg_action = "reject"
        else:
            msg_action = "accept"
        
        return is_okay, msg_action, msg
        
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
    
    def get_response_to_commander(self, directive, msg_type, msg, params):
        """
        Build a response message to the commanding module.
        """
        
        # Don't make a message if we are told to ignore
        if self.ignore:
            response_msg = None
        
        # Create a response message    
        else:
            response_msg = create_response_msg(directive.id, self.module_name,
                                               directive.source, msg_type, msg,
                                               params, None)
        
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
            rospy.logdebug("Issuing default directive")
        
            return self.default_directive
        
    def merge_directives(self):
        """
        Merge the directives stored in the module to get an arbitrated
        directive to send to control.
        
        TODO: Test changes
        """
        
        # Merge the directives to get an arbitrated directive
        rospy.logdebug("Merging directives")
        arb_directive = self.merge_algorithm.run(self.cur_directive, 
                                                 self.directives)
        arb_key = arb_directive.source + "_" + str(arb_directive.id)
        
        # If output key is different, switch to new directive 
        if arb_key != self.dir_key:
            replace = self.cur_directive is not None
            self.dir_key = arb_key
            self.cur_directive = arb_directive
            replace_msg = ""
            
            # Clear stored directives if replacing current directives
            if replace:
                self.directives = {self.dir_key: self.cur_directive}
                replace_msg = ", replacing current directives"
            
            # Update arbitrated directive id number from internal count
            self.cur_id = self.id_count
            self.id_count += 1
            arb_directive.id = self.cur_id
            rospy.logdebug("Switching to directive {} '{}'{}".format(
                self.cur_id, self.cur_directive.name, replace_msg))
        
        # Otherwise continue with current directive 
        else:
            arb_directive = None
            rospy.logdebug("Continuing with directive {} '{}'".format(
                self.cur_id, self.cur_directive.name))
        
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
            is_okay, msg_action, msg = self.process_new_directive(directive)
            
            # Get response to commanding module(s)
            cmdr_msg = self.get_response_to_commander(directive, msg_action,
                                                      msg, None)
            
            # Arbitrate over directives
            if is_okay:
                if not self.ignore:
                    arb_directive = self.merge_directives()
                    self.standby = False
                    
                    # Delete hold-over if it exists
                    #TODO: Eliminate
                    self.hold_over = None
                
                # Reset ignore tag
                else:
                    self.ignore = False
        
        # Handle "held over" directves if we didn't get new one
        # TODO: Eliminate
        elif self.hold_over is not None:
            arb_directive = self.hold_over
            self.standby = False
            self.hold_over = None
        
        # Process new response
        elif response is not None:
            success, msg = self.process_new_response(response)
            
            # Get response to commanding module(s)
            cmdr_msg = self.get_response_to_commander(self.cur_directive,
                                                      msg_action, msg,
                                                      response.params)
            
            # Cleanup the completed/failed directive
            self.directives.pop(self.dir_key)
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
    
    def flush(self):
        """
        Flush all stored directives, but keep the currently executing
        directive.
        """
        
        self.directives = {}
    
    def reset(self):
        """
        Reset the component, including removal of all directive running
        or stored, and setting all variables to their original values.
        """
        
        self.cur_directive = None
        self.standby = True
        self.directives = {}
        self.hold_over = None
        self.cur_id = -1
        self.dir_key = ""
        self.id_count = 1
