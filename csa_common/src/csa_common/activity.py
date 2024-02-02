#!/usr/bin/env python3

"""
  Generic action base class source code.
  
  Copyright 2024 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/csa_ros
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


import rospy


class Activity(object):
    """
    Base class object for activities. Intended for use with the
    'DescreteActivityManager' class in the common applications package.
    """
    
    def __init__(self):
        pass
        
    def get_outputs(self, params):
        """
        
        """
        
        acts_out = {}
        
        return acts_out

