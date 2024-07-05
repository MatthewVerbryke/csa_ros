#!/usr/bin/env python3

"""
  CSA module main module source code.
  
  Copyright 2022-2024 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/csa_ros
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


import os
import sys
import threading

import rospy

from csa_module.activity_manager import ActivityManagerComponent
from csa_module.arbitration import ArbitrationComponent
from csa_module.control import ControlComponent
from csa_msgs.msg import Directive, Response
from csa_msgs.response import create_response_msg
from rosbridge_wrapper.ros_connect_wrapper import RosConnectWrapper as rC


class CSAModule(object):
    """
    A generic CSA type control module object.
    """
    
    def __init__(self, name, arb_algorithm, tact_algorithm, am_algorithm,
                 model, state_topics, pub_topics):
        
        # Get home directory
        self.home_dir = os.getcwd()
        
        # Initialize rospy node
        rospy.init_node(name)
        self.name = name
        
        # Setup cleanup function
        rospy.on_shutdown(self.cleanup)
        
        # Get lock
        self.lock = threading.Lock()
        
        # Get module parameters
        self.subsystem = rospy.get_param("~subsystem", "")
        self.robot = rospy.get_param("~robot", "")
        model_params = rospy.get_param("~model_config", {})
        rate = rospy.get_param("~rate", 30.0)
        self.rate = rospy.Rate(rate)
        max_directives = rospy.get_param("~max_dirs", 2)
        latency = rospy.get_param("~latency", 0.01)
        prefix = rospy.get_param("~prefix", False)
        self.expect_resp = rospy.get_param("~expect_resp", True)
        
        # Add prefix if option selected
        if prefix:
            self.name = self.subsystem + "/" + self.name
        
        # Signal initialization
        rospy.loginfo("'{}': Node initialized".format(self.name))
        
        # Setup system model
        self.setup_model(model, model_params)
        
        # Setup main components
        self.arbitration = ArbitrationComponent(self.name, arb_algorithm,
                                                max_directives)
        self.control = ControlComponent(self.name, tact_algorithm, rate, 
                                        latency, self.model)
        self.activity_manager = ActivityManagerComponent(self.name,
                                                         am_algorithm)
        
        # Signal completion
        rospy.loginfo("'{}': Module components initialized".format(self.name))
        
        # Create empty subscribers callback holding variables
        self.command = None
        self.response = None
        self.state = {}
        
        # Create initial flag variables
        self.got_first_state = False
        
        # Initialize communication objects
        self.initialize_communications(state_topics, pub_topics)
    
    def setup_model(self, model_obj, params):
        """
        Setup the model given to this node as an argument with parameters
        given through the ROSParam server.
        """
        
        # Configure model using parameters
        if params == {}:
            rospy.logerr("'{}': No model parameterization recieved".format(
                self.name))
        else:
            model_obj.configure_model(params)
            
            # Retrieve selected subsystem(s) if needed
            if self.subsystem != "":
                sub_model = model_obj.get_sub_model(self.subsystem)
                self.model = sub_model
            else:
                self.model = model_obj
        
        # Signal Completion
        rospy.loginfo("'{}': System model configured".format(self.name))
    
    def initialize_communications(self, state_topics, pub_topics):
        """
        Initialize the communication interfaces for the module.
        """
        
        # Publisher storage dicts
        self.publishers = {}
        self.pub_adj = {}
        
        # Setup information for default subscriptions
        self.commands_topic = self.name + "/command"
        self.responses_topic = self.name + "/response"
        
        # Initialize command and response subscriptions
        self.command_sub = rospy.Subscriber(self.commands_topic, Directive,
                                            self.command_callback)
        self.response_sub = rospy.Subscriber(self.responses_topic, Response,
                                             self.response_callback)
        
        # Setup state subscription(s)
        for key,value in state_topics.items():
            self.setup_state_subscriber(key, value)
        
        # Setup all required command publishers for other modules
        for key,value in pub_topics.items():
            self.setup_publisher(key, value)
        
        # Signal completion
        rospy.loginfo("'{}': Communication interfaces setup".format(self.name))
        
    def setup_state_subscriber(self, name, config):
        """
        Setup an individual state subscriber.
        """
        
        # State subscriber dict
        self.state_subscribers = {}
        
        # Get basic parameters of subscriber
        topic_type = config["type"]
        prefix_option = bool(config["use_prefix"])
        robot_ns_option = bool(config["use_robot_ns"])
        key = config["key"]
        
        # Handle prefixes for topic
        if robot_ns_option and prefix_option:
            prefix = self.robot + "/" + self.subsystem + "/"
        elif robot_ns_option and not prefix_option:
            prefix = self.robot + "/"
        elif not robot_ns_option and prefix_option:
            prefix = self.subsystem + "/"
        else:
            prefix = ""
        
        # Setup topic name
        topic = prefix + name
        
        # Create subscriber
        sub = rospy.Subscriber(name=topic, data_class=topic_type,
                               callback=self.state_callback,
                               callback_args=key)
        
        # Package into dictionary
        self.state_subscribers.update({key: sub})
        self.state.update({key: None})
    
    def setup_publisher(self, name, config):
        """
        Setup an individual publisher.
        """
        
        # Get basic parameters of publisher
        topic_type = config["type"]
        destination = config["destination"]
        prefix_option = bool(config["use_prefix"])
        robot_ns_option = bool(config["use_robot_ns"])
        
        # Handle prefixes for topic
        if robot_ns_option and prefix_option:
            prefix = self.robot + "/" + self.subsystem + "/"
        elif robot_ns_option and not prefix_option:
            prefix = self.robot + "/"
        elif not robot_ns_option and prefix_option:
            prefix = self.subsystem + "/"
        else:
            prefix = ""
        
        # Setup topic name
        if topic_type == Directive:
            topic = prefix + name + "/command"
        elif topic_type == Response:
            topic = prefix + name + "/response"
        else:
            topic = prefix + name
        
        # Create publisher using rospy ("local") or websockets
        if destination == "local":
            pub = rospy.Publisher(topic, topic_type, queue_size=1)
            pub_type = "rospy"
        else:
            pack_function = value["pack_function"]
            pub = rC.RosMsg(destination, "pub", topic, topic_type, 
                            pack_function)
            pub_type = "ws4py"
        
        # Handle the interface option
        if "interface" in config:
            interface = config["interface"]
        else:
            interface = None
        
        # Create key for storage
        new_key = prefix + name
        
        # Package into dictionary
        pub_dict = {"type": pub_type,
                    "publisher": pub,
                    "interface": interface}
        
        # Add sub-entry into main publishers dict
        self.publishers.update({new_key: pub_dict})
        
    def publish_message(self, msg):
        """
        Publish a message using the correct message passing protocol for
        the desired publisher object.
        """
        
        # Get appropriate publisher option
        if msg.destination in self.pub_adj.keys():
            pub_key = self.pub_adj[msg.destination]
            msg.destination = pub_key
        else:
            pub_key = msg.destination
        
        # Handle interface if needed
        if self.publishers[pub_key]["interface"] is not None:
            msg_to_pub = self.publishers[pub_key]["interface"].convert(msg)
        else:
            msg_to_pub = msg
            
            # Give this module name if necessary
            if msg_to_pub.source == "":
                msg_to_pub.source = self.name
            
        # Publish message over correct protocol
        if self.publishers[pub_key]["type"] == "rospy":
            self.publishers[pub_key]["publisher"].publish(msg_to_pub)
        elif self.publishers[pub_key]["type"] == "ws4py":
            self.publishers[pub_key]["publisher"].send(msg_to_pub)
        
    def publish_multiple_messages(self, msgs):
        """
        Publish multple messages at the same time.
        """
        
        for key,value in msgs.items():
            self.publish_message(value)
    
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
    
    def state_callback(self, msg, args):
        """
        Callback function for state messages to this module. Capable of
        handling messages from multiple state topics simultaneously.
        
        See ROS Answers question #407730: 'ROS Subscribe to multiple
        topics with single function'
        """
        
        # Catagorize and store incoming response message
        self.lock.acquire()
        key = args
        self.state[key] = msg
        self.lock.release()
    
    def run_once(self):
        """
        Run the components of the module in the proper order once.
        """
        
        # Check if we have new directive/command
        arb_output = self.arbitration.run(self.command, None)
        arb_directive = arb_output[0]
        arb_response = arb_output[1]
        
        # Response to comanding module (if necessary)
        if arb_response is not None:
            self.publish_message(arb_response)
        
        # Check for completion when not expecting external responses
        if not self.expect_resp and self.control.tactic is not None:
            self.check_for_completion()
        
        # Check for new response(s)
        am_output = self.activity_manager.run(None, self.response)
        am_directives = am_output[0]
        am_responses = am_output[1]
        
        # Respond to commanded modules (if necessary)
        if am_directives is not None:
            self.publish_multiple_message(am_directives)
        
        # Run Control
        ctrl_output = self.control.run(arb_directive, am_responses, self.state)
        ctrl_directive = ctrl_output[0]
        ctrl_response = ctrl_output[1]
        
        # Run activity manager
        am_output = self.activity_manager.run(ctrl_directive, None)
        am_directives = am_output[0]
        am_response = am_output[1]
        
        # Issue command(s)
        if am_directives is not None:
            self.publish_multiple_messages(am_directives)
        
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
        rospy.loginfo("'{}': Node is running...".format(self.name))
        while not rospy.is_shutdown():
            self.run_once()
            self.rate.sleep()
        
    def check_for_completion(self):
        """
        Handling function for tactics when module is not recieving
        responses to its outputs (typically because its using an 
        interface). Creates a resposne message which elicits the correct
        response from the module.
        """
        
        # Get relevant information from control component
        completion = self.control.tactic.completion
        fail_msg = self.control.tactic.fail_msg
        dir_id = self.control.cur_id
        
        # Create appropriate response message for situation
        if completion == "in progress":
            pass
        elif completion == "complete":
            self.response = create_response_msg(dir_id, "", self.name,
                                                "success", fail_msg, None, "")
        elif completion == "failed":
            self.response = create_response_msg(dir_id, "", self.name,
                                                "failure", fail_msg, None, "")
    
    def cleanup(self):
        """
        Things to do when shutdown occurs.
        """
        
        # Log shutdown of module
        rospy.sleep(1)
        rospy.loginfo("'{}': Node shutting down".format(self.name))
