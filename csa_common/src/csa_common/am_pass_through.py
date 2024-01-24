#!/usr/bin/env python3

"""
  Pass-through activity manager algorithm source code.
  
  Copyright 2023 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/csa_ros
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


import copy

import rospy

from csa_module.activity_manager_algorithm import ActivityManagerAlgorithm
from csa_msgs.msg import Directive, Response


class PassThroughActivityManager(ActivityManagerAlgorithm):
    """
    A very simple activity manager algorithm which simply passes through
    directives to be published.
    
    NOTE: Only to be used when sending one directive per destination.
    """
    
    def __init__(self):
        super().__init__()
        
    def run(self, directives):
        """
        Simply return the directives
        
        TODO: check to make sure we aren't overloaded?
        """
        
        # Copy direcitve list
        directives_out = copy.deepcopy(directives)
        
        return directives_out, True, ""

            
