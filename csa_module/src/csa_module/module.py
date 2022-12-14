#!/usr/bin/env python3

"""
  CSA module main module source code.
  
  Copyright 2022 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/csa_ros
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
    A generic CSA type module object.
    """
    
    def __init__(self, name, arb_algorithm, tact_algorithm):
        
        # Get home directory
        self.home_dir = os.getcwd()
        
        # Initialize rospy node
        rospy.init_node(name)
        rospy.loginfo("'%s' node initialized", name)
        self.name = name
        
        # Setup cleanup function
        rospy.on_shutdown(self.cleanup)
        
        # Get a lock
        self.lock = threading.Lock()
        
        # Get module parameters
        self.rate = rospy.Rate(rospy.get_param("~rate", 30))
        max_directives = rospy.get_param("~max_dirs", 2)
        allowed_list = rospy.get_param("~allowed_list", [])
        default_name = rospy.get_param("~default", "")
        allowed_order = rospy.get_param("~order", [])
        latency = rospy.get_param("~latency", 0.01)
        tolerance = rospy.get_param("~tolerance", 0.1)
        state_topic = rospy.get_param("~state_topic", "")
        pub_topics = rospy.get_param("~pub_topics", {})
        
        # Setup the components
        self.arbitration = ArbitrationComponent(self.name, arb_algorithm,
                                                default_name, allowed_list,
                                                allowed_order, max_directives)
        self.control = ControlComponent(self.name, tact_algorithm, latency,
                                        tolerance)
        #TODO: Activity Manager
        
        # Create empty subscribers callback holding variables
        self.command = None
        self.response = None
        self.state = None
        
        # Signal completion
        rospy.loginfo("Module components initialized")
    
        # Initialize communication objects
        self.initialize_communciations(state_topic, pub_topics)
    
    def initialize_communications(self, state_topic, pub_topics):
        """
        Initialize the communication interfaces for the module.
        
        TODO: Test ws4py publishers
        """
        
        # Publisher storage dicts
        self.publishers = {}
        self.pub_types = {}
        
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
            topic_type = value["type"]
            destination = value["destination"]
            
            # Get topic name if allowed type
            if topic_type == "Directive":
                topic = key + "/command"
            elif topic_type == "Response":
                topic = key + "/response"
            elif topic_type == "Other":
                topic = "TODO"
            else:
                rospy.logerr("Topic type '{}' not recognized".format(topic_type))
                exit()
                
            # Create publishers using rospy ("local") or websockets
            if destination == "local":
                pub = rospy.Publisher(topic, topic_type, queue_size=1)
                pub_type = "rospy"
            else:
                pub = rC.RosMsg(destination, "pub", topic, topic_type, None) #TODO <-- add packing function
                pub_type = "ws4py"
            
            # Add to storage dictionary
            self.publishers.update({key: pub})
            self.pub_types({key: pub_type})
        
        # Signal completion
        rospy.loginfo("Communication interfaces setup")
        
    def publish_message(self, msg):
        """
        Publish a message using the correct message passing protocol for
        the desired publisher object.
        """
        
        # Get publisher info from the message
        destination = msg.destination
        pub_type = self.pub_types[destination]
        
        # Publish message over the right protocol
        if pub_type == "rospy":
            self.publishers[pub_key].publish(msg)
        elif pub_type == "ws4py":
            self.publishers[pub_key].send(msg)
    
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
        
        # Check if we have new directive/command
        arb_output = self.arbitration.run(self.command, None)
        arb_directive = arb_output[0]
        arb_response = arb_output[1]
        
        # Response to comanding module (if necessary)
        if arb_response is not None:
            self.publish_message(arb_response)
        
        # Check for new response
        # TODO: Run activity manager
        
        # Run Control
        ctrl_output = self.control.run(arb_directive, self.response, self.state)
        ctrl_directive = ctrl_output[0]
        ctrl_response = ctrl_output[1]
        #TODO: Run activity manager
        
        # Issue command(s)
        if ctrl_directive is not None:
            self.publish_message(ctrl_directive)
            rospy.loginfo("Issuing directive %s to '%s'", ctrl_directive.id,
                destination)
        
        # Respond to commanding module if necessary
        if ctrl_response is not None:
            arb_output = self.arbitration.run(None, ctrl_response)
            arb_response = arb_output[1]
            self.publish_message(arb_response)
            
        # Purge command and response callbacks for next loop
        self.command = None
        self.response = None
        
    def run(self):
        """
        Keep looping through the module while rospy is running.
        """
        
        # Main loop
        rospy.loginfo("'%s' node is running...", self.name)
        while not rospy.is_shutdown():
            self.run_once()
            self.rate.sleep()
        
    def cleanup(self):
        """
        Things to do when shutdown occurs.
        """
        
        # Log shutdown of module
        rospy.sleep(1)
        rospy.loginfo("Shutting down '%s' node", self.name)
