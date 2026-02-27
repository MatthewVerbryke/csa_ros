#!/usr/bin/env python3

"""
  Discrete action activity manager algorithm source code.
  
  Copyright 2024 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/csa_ros
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


import copy

from csa_common.activity import Activity
from csa_common.inert import InertActivity
from csa_module.activity_manager_algorithm import ActivityManagerAlgorithm


class DiscreteActivityManager(ActivityManagerAlgorithm):
    """
    An activity manager for a preselected set of discrete activities to
    execute.
    """
    
    def __init__(self, act_list, dest_names, use_prefix):
        
        # Set response expectation parameter
        expect_resp = True
        super().__init__(expect_resp)
        
        # Initialize other parameters and variables
        self.use_prefix = use_prefix
        self.dest_names = dest_names
        self.act_dict = {}
        self.allowed_names = []
        self.req_responses = {}
        self.id_count = -1
        
        # Create and store 'inert' activity
        dests = list(self.dest_names.values())
        self.act_dict.update({"inert": InertActivity(dests)})
        self.allowed_names.append("inert")
        
        # Store other actions in dictionary and allowed list
        for act in act_list:
            self.act_dict.update({act.name: act})
            self.allowed_names.append(act.name)
    
    def adjust_dest_names(self, prefix):
        """
        Adjust the names of destination modules with the given prefix, 
        if specified in the module initialization script.
        """
       
        # Correct destination names list
        for key,value in self.dest_names.items():
            new_name = prefix + "/" + value
            self.dest_names[key] = new_name
        
        # Correct desitnations within activities
        for key,value in self.act_dict.items():
            new_act_dests = []
            for dest in value.dests:
                new_dest = prefix + "/" + dest
                new_act_dests.append(new_dest)
            self.act_dict[key].dests = new_act_dests
    
    def process_response(self, response):
        """
        Evaluate a response message to current output directives and 
        determine an appropriate overall response for the control
        directive.
        """
        
        # Get out parameters
        params = response.params
        
        # If 'accept' meassage, continue
        if response.status == "accept":
            mode = "continue"
        
        # If successful determine if continue or finish
        elif response.status == "success":
            mode = self.activity.check_response(response)
        
        # Otherwise fail up to control
        else:
            mode = "failure"
        
        return mode, params
    
    def execute_activity(self, directive):
        """
        Given a control directive, check the requested activity and, if
        valid, create a set of output direcitves.
        """
        
        msg = ""
        success = False
        directives_out = {}
        
        # Determine if activity allowed
        allowed = False
        for name in self.allowed_names:
            if directive.name == name:
                allowed = True
        
        # Handle non-allowed activity
        if not allowed:
            msg = "Activity {} not recognized".format(directive.name)
        
        # Determine output directives
        else:
            self.activity = copy.deepcopy(self.act_dict[directive.name])
            activities = self.activity.get_outputs(directive.params)
            success = True
            
            # Package output directives with new ids
            if success:
                for act in activities:
                    self.id_count += 1
                    act.id = self.id_count
                    directives_out.update({act.id: act})
            
                # Store outputs for later tracking
                self.cur_directives = directives_out
        
        return directives_out, success, msg
    
    def clear_stored_directives(self):
        """
        Clear the stored list of currently executing output directives 
        and main activity.
        """
        self.activity = None
        self.cur_directives = {}
