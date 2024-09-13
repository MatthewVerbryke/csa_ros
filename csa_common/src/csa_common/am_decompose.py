#!/usr/bin/env python3

"""
  Multi-directive Pass-through activity manager algorithm source code.
  
  Copyright 2023 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/csa_ros
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


from csa_module.activity_manager_algorithm import ActivityManagerAlgorithm
from csa_msgs.directive import create_directive_msg
from csa_msgs.msg import Directive, Response
from csa_msgs.params import create_param_submsg, convert_params_to_dict


class DecomposedActivityManager(ActivityManagerAlgorithm):
    """
    A simple activity manager algorithm which splits a single directive
    along tactic specified lines and then sends them to be published
    through directives with no response expected.
    
    NOTE: Only to be used when sending one directive per destination
    NOTE: Assumes some level of symmetry in outputs (i.e. all outputs
          will use the same directive name, description, priority, etc., 
          even if id, destinantion and parameters may be different)
    
    TODO: Test
    """
    
    def __init__(self, dests, source):
        
        # Set response expectation parameter
        expect_resp = False
        super().__init__(expect_resp)
        
        # Initialize id count variable
        self.id_count = -1
        self.dests = dests
        self.source = source
        
    def process_response(self, response):
        """
        Simple return status and params of response
        """
        
        # Copy relevant response elements
        mode = response.status
        params = response.params
        
        return mode, params
        
    def execute_activity(self, directive):
        """
        Convert a single directive into multiple output directives, 
        based on destination tags.
        """
        
        directives_out = {}
        success = True
        msg = ""
        
        # Store elements which won't be split
        name = directive.name
        description = directive.description
        t_resp = directive.response_time.to_sec()
        deadline = directive.params.deadline
        priority = directive.priority
             
        # Convert parameters back to python types
        full_params = convert_params_to_dict(directive.params)
        
        # Get parameters
        for dest in self.dests:
            if full_params["entry_conds"] != {}:
                entry_conds = full_params["entry_conds"][dest]
            else: 
                entry_conds = {}
            
            if full_params["end_conds"] != {}:
                end_conds = full_params["end_conds"][dest]
            else:
                entry_conds = {}
            
            if full_params["criteria"] != {}:
                criteria = full_params["criteria"][dest]
            else:
                criteria = {}
                
            if full_params["rules"] != {}:
                rules = full_params["rules"][dest]
            else:
                rules = {}
            
            # Create directive for this destination
            self.id_count += 1
            params = create_param_submsg(entry_conds, end_conds, criteria, rules,
                                         deadline)
            directive_out = create_directive_msg(self.id_count, name, 
                                                 description, self.source, dest,
                                                 t_resp, priority, params, "")
                                                 
            # Give directive to output dict
            directives_out.update({directive_out.id: directive_out})
        
        return directives_out, success, msg
