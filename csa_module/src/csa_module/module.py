#!/usr/bin/env python

"""
  CSA module main module python source code.
  
  Copyright 2022 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/gazebo_terrain
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


import os
import sys
import threading

import rospy

from csa_module.arbitration import ArbitrationComponent
from csa_module.control import ControlComponent
from csa_msgs.msg import Directive, Response
from csa_module.tactics import TacticsComponent


class CSAModule(object):
    """
    A generic CSA type module object. This is not meant to be run
    independently, but instead, be used as an inherited class.
    
    TODO: Re-Test
    """
    
    def __init__(self):
        
        # Get home directory
        self.home_dir = os.getcwd()
        
        # Initialize rospy node
        rospy.init_node(name)
        rospy.loginfo("'{}' node initialized".format(name))
        
        # Setup cleanup function
        rospy.on_shutdown(self.cleanup)
        
        # Get a lock
        self.lock = threading.Lock()
        
        # Get module parameters
        self.name = rospy.get_param("~name", "")
        self.rate = rospy.get_param("~rate", 100.0)
        self.system = rospy.get_param("~robot", "")
        self.ref_frame = rospy.get_param("~reference_frame", "world")
        
        # Get the necessary component functions
        arb_algorithm = rospy.get_param("arb_function")
        tact_algorithm = rospy.get_param("tact_function")
        
        # Setup the components
        self.arbitration = ArbitrationComponent(name, arb_algorithm)
        self.control = ControlComponent(tact_algorithm)
        #self.tactics = TacticsComponent(tactics_algorithm)
        #TODO: Activity Manager
        
        # Create empty subscribers callback holding variables
        self.command = None
        self.response = None
        self.state = None
        
        # Signal completion
        rospy.loginfo("Module components initialized")
        
        # start runing module
        self.run()
        
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
        
        # Setup state information topic
        for key,value in state_topic.items():
            self.state_topic = key
            self.state_format = value
        
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
        for key,value in pub_topics.items():
            if value == Directive:
                topic = key + "/commands"
                entry = {key: rospy.Publisher(topic, Directive, queue_size=1)}
            elif value == Response:
                topic = key + "/responses"
                entry = {key: rospy.Publisher(topic, Response, queue_size=1)}
                                              
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
        Run the components of the module in the proper order.
        """
        
        # Check if we have a new directive/command
        arb_output = self.arbitration.run(self.command)
        arb_directive = arb_output[0]
        arb_response = arb_output[1]
        
        # Response to comanding module (if necessary)
        if arb_directive is not None:
            destination = arb_repsonse.destination
            self.publishers[destination].publisher(arb_response)
        
        # Check for a new response
        # TODO: Run activity manager

        # Run Control
        ctrl_output = self.control.run(arb_directive, self.response, self.state)
        ctrl_directive = ctrl_output[0]
        ctrl_response = ctrl_output[1]
        
        #TODO: Run activity manager
        
        # Issue command(s)
        if ctrl_directive is not None:
            destination = ctrl_directive.destination
            self.publishers[destination].publisher(ctrl_directive)
        
        # Respond to commanding module if necessary
        if ctrl_response is not None:
            destination = ctrl_response.destination
            self.publishers[destination].publisher(ctrl_response)
        
        # Purge command and response callbacks for next loop
        self.command = None
        self.response = None
        
    def cleanup(self):
        """
        Things to do when shutdown occurs.
        """
        
        # Log shutdown of the module
        rospy.sleep(1)
        rospy.loginfo("Shutting down '{}' node".format(self.name))
