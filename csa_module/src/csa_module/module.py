#!/usr/bin/env python

"""
  CSA module main module python source code.
  
  Copyright 2021 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/gazebo_terrain
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


import os
import sys
import _thread as thread

import rospy

from arbitration import ArbitrationComponent
from control import ControlComponent
import csa_msg.msg import Directive, Response
from tactics import TacticsComponent


class CSAModule(object):
    """
    A generic CSA type module object. This is not meant to be run
    independently, but instead, be used as an inherited class.
    
    TODO: Test
    """
    
    def __init__(self, name, functions):
        
        # Get home directory
        self.home_dir = os.getcwd()
        
        # Initialize rospy node
        rospy.init_node(name)
        rospy.loginfo("'{}' node initialized".format(name))
        
        # Get module parameters
        self.name = name
        self.rate = rospy.get_param("~rate", 100.0)
        self.system = rospy.get_param("~robot", "")
        self.ref_frame = rospy.get_param("~reference_frame", "world")
        
        # Get the necessary component functions
        arb_algorithm = functions["arbitration"]
        tactics_algorithm = functions["tactic_selection"]
        
        # Setup the components
        self.arbitration = ArbitrationComponent(arb_algorithm)
        self.control = ControlComponent()
        self.tactics = TacticsComponent(tactics_algorithm)
        
        # Get a lock
        self.lock = thread.allocate_lock()
        
        # Create empty module
        self.arb_to_ctrl = None
        self.ctrl_to_arb = None
        self.ctrl_to_tact = None
        self.tact_to_ctrl = None
        
        # Setup cleanup function
        rospy.on_shutdown(self.cleanup)
        
    def main(self, directives, responses, state):
        """
        Run the components of the module once.
        
        TODO: Investigate running them in parallel
        TODO: Test
        """
            
        # Run each component of the module
        arb_output = self.arbitration.run(directives,
                                          self.ctrl_to_arb)
        ctrl_output = self.control.run(self.arb_to_ctrl,
                                       self.tact_to_ctrl,
                                       state,
                                       response)
        tact_output = self.tactics.run(self.ctrl_to_tact)
        
        # Store output messages needed for the next loop
        self.arb_to_ctrl = arb_output[0]
        self.ctrl_to_arb = ctrl_output[0]
        self.ctrl_to_tact = ctrl_output[1]
        self.tact_to_ctrl = tact_output
        
        # Output messages for other modules
        response = arb_output[1]
        directive = ctrl_output[2]
        
        return response, directive
            
    def cleanup(self):
        """
        Things to do when shutdown occurs.
        """
        
        # Log shutdown of the module
        rospy.sleep(1)
        rospy.loginfo("Shutting down node '{}'".format(self.name))
