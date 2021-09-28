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
from csa_msgs.msg import Directive, Response
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
        
        # Setup cleanup function
        rospy.on_shutdown(self.cleanup)
        
        # Get a lock
        self.lock = thread.allocate_lock()
        
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
        #TODO: Activity Manager
        
        # Create empty subscribers callback holding variables
        self.command = None
        self.response = None
        self.state = None
        
        # Create empty internal messages holding variables
        self.arb_to_ctrl = None
        self.ctrl_to_arb = None
        self.ctrl_to_tact = None
        self.tact_to_ctrl = None
        
        # Signal completion
        rospy.loginfo("CSA module components initialized")
        
    def initialize_communications(self, state_topic, pub_topics):
        """
        Initialize the communication interfaces for the module. 
        
        TODO: catch errors
        TODO: include ROSbridge communcation patterns.
        """
        
        # Publisher storage
        self.publishers = {}
        
        # Setup information for default subscriptions
        self.commands_topic = self.name + "/commands"
        self.responses_topic = self.name + "/responses"
        self.state_topic = state_topic[0]
        self.state_format = state_topic[1]
        
        # Initialize common subscriptions
        self.command_sub = rospy.Subscriber(self.commands_topic,
                                            Directive,
                                            self.command_callback)
        self.response_sub = rospy.Subscriber(self.responses_topic,
                                             Response,
                                             self.response_callback)
        self.state_sub = rospy.Subscriber(self.state_topic,
                                          self.state_format,
                                          self.state_callback)
        
        # Setup all required publishers
        for key,value in pub_topics:
            if value == "Directive":
                entry = {key: rospy.Publisher(key, Directive, queue_size=1)}
            elif value == "Response":
                entry = {key: rospy.Publisher(key, Response, queue_size=1)}
                                              
            # Add to storage dictionary
            self.publishers.update(entry)
        
        # Signal completion
        rospy.loginfo("Communication interfaces setup")
        
    def command_callback(self, msg):
        """
        Callback function for directive/command messages to this module.
        """
        
        # Store incoming command messages
        self.lock.acquire()
        self.command = msg
        self.lock.release()
        
    def response_callback(self, msg):
        """
        Callback function for response messages to this module.
        """
        
        # Store incoming command messages
        self.lock.acquire()
        self.response = msg
        self.lock.release()
        
    def state_callback(self, msg):
        """
        Callback function for state messages from the state estimator.
        """
        
        # Store incoming command messages
        self.lock.acquire()
        self.state = msg
        self.lock.release()
        
    def run(self):
        """
        Run the components of the module once.
        
        TODO: Investigate running them in parallel
        """
        
        # Run each component of the module
        arb_output = self.arbitration.run(directives,
                                          self.ctrl_to_arb)
        ctrl_output = self.control.run(self.arb_to_ctrl,
                                       self.tact_to_ctrl,
                                       state,
                                       responses)
        tact_output = self.tactics.run(self.ctrl_to_tact)
        
        # Store output messages needed for the next loop
        self.arb_to_ctrl = arb_output[0]
        self.ctrl_to_arb = ctrl_output[0]
        self.ctrl_to_tact = ctrl_output[1]
        self.tact_to_ctrl = tact_output
        
        # Get the output messages and relevant information
        response = arb_output[1]
        response_dest = response.destination
        directive = ctrl_output[2]
        directive_dest = directive.destination
        
        # Publish messages
        # TODO: handle multiple messages of same type to multiple
        #       locations
        self.publishers[directive_dest].publisher(directive)
        self.publishers[response_dest].publisher(response)
        
    def cleanup(self):
        """
        Things to do when shutdown occurs.
        """
        
        # Log shutdown of the module
        rospy.sleep(1)
        rospy.loginfo("Shutting down node '{}'".format(self.name))
