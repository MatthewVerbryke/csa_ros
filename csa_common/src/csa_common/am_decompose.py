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
from csa_msgs.params import create_param_submsg


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
    
    def __init__(self, dests):
        
        # Set response expectation parameter
        expect_resp = False
        super().__init__(expect_resp)
        
        # Initialize id count variable
        self.id_count = -1
        self.dests = dests
        
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
        t_resp = directive.t_resp
        deadline = directive.params.deadline
        priority = directive.priority
        
        # convert parameters back to python types
        full_entry_conds = ast.literal_eval(directive.params.entry_conds)
        full_end_conds = ast.literal_eval(directive.params.end_conds)
        full_criteria = ast.literal_eval(directive.params.criteria)
        full_rules = ast.literal_eval(directive.params.rules)
        
        # Get parameters
        for dest in self.dests:
            entry_conds = full_entry_conds[dest]
            end_conds = full_end_conds[dest]
            criteria = full_criteria[dest]
            rules = full_rules[dest]
            
            # create directive for this destination
            self.id_count += 1
            params = create_param_submsg(entry_conds, end_conds, criteria, rules,
                                         deadline)
            directive_out = create_directive_msg(self.id_count, name, 
                                                 description, self.module_name,
                                                 dest, t_resp, priority, params,
                                                 "")
                                                 
            # Give directive to output dict
            directives_out.update({directive_out.id: directive_out})
        
        return directives_out, success, msg
