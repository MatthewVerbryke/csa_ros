#!/usr/bin/env python3

"""
  Generic CSA tactic selection algorithm base class source code.
  
  Copyright 2023 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/csa_ros
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


class TacticsAlgorithm(object):
    """
    Base class object for a tactic selection algorithm.
    """
    
    def __init__(self, tactic_list):
        
        # Store list of tactics which can be called
        self.tactic_list = tactic_list
        
        # TODO: add checks for this ^?
        
    def get_module_name(self, module_name):
        """
        Store the name of the module (used to give a module source to 
        directives)
        """
        self.module_name = module_name
    
    def run(self, directive, state, model):
        """
        Run the tactics selection algorithm (needs to be filled end by
        user).
        """
        
        tactics = [""]
        status = True
        
        return success, tactics
