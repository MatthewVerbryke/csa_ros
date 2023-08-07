#!/usr/bin/env python3

"""
  Generic CSA activity manager algorithm base class source code.
  
  Copyright 2021-2023 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/gazebo_terrain
  Additional copyright may be held by others, as reflected in the commit
  history.
  
  TODO: Test component
"""


import rospy

from csa_msgs.msg import Directive, Response
from csa_msgs.response import create_response_msg
from csa_msgs.directive import create_directive_msg


class ActivityManagerAlgorithm(object):
    """
    Base class object for activitiy manager algorithms.
    """
    
    def __init__(self):
        pass
    
    def run(self, directives):
        
        directives = [Directive()]
        
        return directives
