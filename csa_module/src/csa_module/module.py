#!/usr/bin/env python3

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


class CSAModule(object):
    """
    A generic CSA type module object. This is not meant to be run
    independently, but instead, be used as an inherited class.
    """
    
    def __init__(self, name, rate, arb_algorithm, tact_algorithm):
        
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
        self.name = name
        self.rate = rospy.Rate(rate)
        
        # Setup the components
        self.arbitration = ArbitrationComponent(self.name, arb_algorithm)
        self.control = ControlComponent(self.name, tact_algorithm)
        #TODO: Activity Manager
        
        # Create empty subscribers callback holding variables
        self.command = None
        self.response = None
        self.state = None
        
        # Signal completion
        rospy.loginfo("Module components initialized")
        
    def initialize_communications(self, state_topic, pub_topics):
        """
        Initialize the communication interfaces for the module. 
        
        TODO: catch errors
        TODO: include ROSbridge communcation patterns.
        """
        
        # Publisher storage
        self.publishers = {}
        
        # Setup information for default subscriptions
        self.commands_topic = self.name + "/command"
        self.responses_topic = self.name + "/response"
        
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
        
        # Setup all required command publishers for other modules
        for key,value in pub_topics.items():
            if value == Directive:
                topic = key + "/command"
                entry = {key: rospy.Publisher(topic, Directive, queue_size=1)}
            elif value == Response:
                topic = key + "/response"
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
        
        # Store incoming response messages
        self.lock.acquire()
        self.response = msg
        self.lock.release()
        
    def state_callback(self, msg):
        """
        Callback function for state messages from the state estimator.
        """
        
        # Store incoming state messages
        self.lock.acquire()
        self.state = msg
        self.lock.release()
        
    def run_once(self):
        """
        Run the components of the module in the proper order once.
        
        TODO: Rework this section
        """
        
        # Check if we have a new directive/command
        arb_output = self.arbitration.run(self.command, None)
        arb_directive = arb_output[0]
        arb_response = arb_output[1]
        
        # Response to comanding module (if necessary)
        if arb_directive is not None:
            destination = arb_response.destination
            self.publishers[destination].publish(arb_response)
        
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
            self.publishers[destination].publish(ctrl_directive)
        
        # Respond to commanding module if necessary
        if ctrl_response is not None:
            arb_output = self.arbitration.run(None, ctrl_response)
            arb_response = arb_output[1]
            destination = arb_response.destination
            self.publishers[destination].publish(arb_response)
            
        # Purge command and response callbacks for next loop
        self.command = None
        self.response = None
        
    def run(self):
        """
        Keep looping through the module while rospy is running
        """
        
        # Main loop
        rospy.loginfo("'{}' node is running...".format(self.name))
        while not rospy.is_shutdown():
            self.run_once()
            self.rate.sleep()
        
    def cleanup(self):
        """
        Things to do when shutdown occurs.
        """
        
        # Log shutdown of the module
        rospy.sleep(1)
        rospy.loginfo("Shutting down '{}' node".format(self.name))
