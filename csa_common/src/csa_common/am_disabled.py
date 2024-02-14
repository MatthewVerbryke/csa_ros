#!/usr/bin/env python3

"""
  Disabled activity manager algorithm source code.
  
  Copyright 2024 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/csa_ros
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


from csa_module.activity_manager_algorithm import ActivityManagerAlgorithm


class DisabledActivityManager(ActivityManagerAlgorithm):
    """
    A "dummy" activity manager which "place-holds" the AM spot in a
    module but has no functionality otherwise. Used for modules which 
    only respond to their commander but have no directive/command
    output.
    
    TODO: Test
    """
    
    def __init__(self):
        
        # Set response expectation parameter
        expect_resp = False
        super().__init__(expect_resp)
        
    def execute_activity(self, directive):
        
        directives = []
        success = True
        msg = ""
        
        return directives, success, msg
        
