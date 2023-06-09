#!/usr/bin/env python3

"""
  Generic CSA tactic base class source code.
  
  Copyright 2023 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/csa_ros
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


from csa_msgs.directive import Directive


class Tactic(object):
    """
    Base class object for a CSA tactic.
    """
    
    def __init__(self, params, model):
        
        self.params = params
        self.model = model
        
    def run(self, state):
        """
        Run the tactic and get a list of directives out (needs to be 
        filled end by user).
        """
        
        directives = [Directive()]
        status = True
        
        return status, directives
