#!/usr/bin/env python3

"""
  Generic CSA tactic base class source code.
  
  Copyright 2023 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/csa_ros
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


import rospy

from csa_msgs.directive import create_directive_msg
from csa_msgs.msg import Directive


class Tactic(object):
    """
    Base class object for a CSA tactic.
    """
    
    def __init__(self, module_name, params, model):
        
        # Initialize variables
        self.completion = "in progress"
        self.fail_msg = ""
        
        # Store parameters
        self.module_name = module_name
        self.params = params
        self.continuous = False
        self.resp_output = False
        self.evals_resp = False
        self.model = model
        self.cur_id = int(params["rules"]["id"])
    
    def run(self, state):
        """
        Run the tactic and get a list of directives out (needs to be 
        filled end by user).
        """
        
        directive = Directive()
        status = True
        
        return status, directive
        
    def evaluate_response(self, response):
        """
        Evaluate response w.r.t. this tactic. This is used to determine
        the current execution status of multi-staged tactics, such as to
        continue or declare success. Not needed for simple one-step 
        tactics where success/failure is enough.
        
        NOTE: Will only run if 'self.eval_resp' is set to 'True'.
        """
        
        mode = "failure"
        rospy.logerr("No response evaluation function specified for tactic")
        
        return mode
