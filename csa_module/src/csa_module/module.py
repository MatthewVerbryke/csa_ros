#!/usr/bin/env python3

"""
  CSA module main module source code.
  
  Copyright 2022-2025 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/csa_ros
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


import os
import sys
import threading

from diagnostic_msgs.msg import DiagnosticStatus, DiagnosticArray
import rospy
from std_msgs.msg import String

from csa_module.activity_manager import ActivityManagerComponent
from csa_module.arbitration import ArbitrationComponent
from csa_module.control import ControlComponent
from csa_msgs.directive import create_directive_obj_from_msg, DirectiveObj
from csa_msgs.msg import Directive, Response, ModuleStatus
from csa_msgs.response import (
    create_response_obj, create_response_obj_from_msg, ResponseObj
)
from csa_msgs.status import create_module_status_msg
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
        rospy.loginfo("'{}' node initialized".format(self.name))
        
        # Setup system model
        self.setup_model(model, model_params)
        
        # Setup main components
        self.arbitration = ArbitrationComponent(self.name, arb_algorithm,
                                                max_directives)
        self.control = ControlComponent(self.name, tact_algorithm, rate, 
                                        latency, self.model)
        self.activity_manager = ActivityManagerComponent(self.name,
                                                         am_algorithm,
                                                         self.subsystem)
        
        # Signal completion
        rospy.logdebug("Module components initialized")
        
        # Create empty subscribers callback holding variables
        self.command = None
        self.response = None
        self.meta_command = None
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
            rospy.logerr("'{}' has no model parameterization".format(self.name))
        else:
            model_obj.configure_model(params)
            
            # Retrieve selected subsystem(s) if needed
            if self.subsystem != "":
                sub_model = model_obj.get_sub_model(self.subsystem)
                self.model = sub_model
            else:
                self.model = model_obj
        
        # Signal Completion
        rospy.logdebug("System model configured")
    
    def initialize_communications(self, state_topics, pub_topics):
        """
        Initialize the communication interfaces for the module.
        """
        
        # Publisher storage dicts
        self.publishers = {}
        self.pub_alt = {}
        
        # Setup information for default subscriptions
        self.commands_topic = self.name + "/command"
        self.responses_topic = self.name + "/response"
        self.meta_command_topic = self.name + "/meta_command"
        self.status_topic = self.name + "/status"
        
        # Initialize command, response, and meta-command subscriptions
        self.command_sub = rospy.Subscriber(self.commands_topic, Directive,
                                            self.command_callback)
        self.response_sub = rospy.Subscriber(self.responses_topic, Response,
                                             self.response_callback)
        self.meta_command_sub = rospy.Subscriber(self.meta_command_topic, String,
                                                 self.meta_command_callback)
        
        # Setup state subscription(s)
        self.state_subscribers = {}
        for key,value in state_topics.items():
            self.setup_state_subscriber(key, value)
        
        # Setup module status publisher
        # TODO: Add rosbridge option?
        self.status_pub = rospy.Publisher(self.status_topic, ModuleStatus,
                                          queue_size=1)
        
        # Setup all required command publishers for other modules
        for key,value in pub_topics.items():
            self.setup_publisher(key, value)
        
        # Signal completion
        rospy.logdebug("Communication interfaces setup")
        
    def setup_state_subscriber(self, name, config):
        """
        Setup an individual state subscriber.
        """
        
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
        
        # Create full name store if needed
        if prefix_option:
            full_dest = self.subsystem + "/" + name
            self.pub_alt.update({full_dest: name})
        
        # Package into dictionary
        pub_dict = {"type": pub_type,
                    "publisher": pub,
                    "interface": interface}
        
        # Add sub-entry into main publishers dict
        self.publishers.update({name: pub_dict})
        
    def publish_message(self, msg_in):
        """
        Publish a message using the correct message passing protocol for
        the desired publisher object.
        """
        
        # Convert to actual ROS message if necessary
        if type(msg_in) == DirectiveObj or type(msg_in) == ResponseObj:
            msg = msg_in.to_msg()
        else:
            msg = msg_in
        
        # Get appropriate publisher option
        if msg.destination != "":
            if msg.destination in self.publishers.keys():
                pub_key = msg.destination
            elif msg.destination in self.pub_alt.keys(): #FIXME?
                pub_key = self.pub_alt[msg.destination]
            else:
                error_msg = "Directive {} destination not recognized".format(msg.id)
                rospy.logwarn("{} {} for '{}'".format(error_msg, msg.destination, self.name))
        
            # Handle interface if needed
            if self.publishers[pub_key]["interface"] is not None:
                msg_to_pub = self.publishers[pub_key]["interface"].convert(msg)
            else:
                msg_to_pub = msg
                
                # Give this module name if necessary
                if msg_to_pub.source == "":
                    msg_to_pub.source = self.name
        
        # If no destination given, set 'msg_to_pub' as None
        # TODO: somewhat hacky, fix?
        else:
            msg_to_pub = None
            if msg.name != "inert":
                msg = "No destination given for directive {}".format(msg.id)
                rospy.logwarn("{} in '{}'".format(msg, self.name))
        
        # Publish message over correct protocol (if message exists)
        if msg_to_pub == None:
            pass
        else:
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
            
    def publish_status_message(self, start_t, end_t):
        """
        Publish a module status message for monitoring the modules
        status.
        """
        
        # Create module status message
        msg = create_module_status_msg(
            self.name,
            start_t,
            end_t,
            self.arbitration.cur_directive, 
            self.control.tactic.name,
            list(self.activity_manager.cur_directives.keys())
        )
                                
        # Publish message
        # TODO: add option for rosbridge publishing?
        self.status_pub.publish(msg)
        
    def command_callback(self, msg):
        """
        Callback function for directive/command messages to this module.
        """
        
        self.lock.acquire()
        self.command = create_directive_obj_from_msg(msg)
        self.lock.release()
    
    def response_callback(self, msg):
        """
        Callback function for response messages to this module.
        """
        
        self.lock.acquire()
        self.response = create_response_obj_from_msg(msg)
        self.lock.release()
    
    def meta_command_callback(self, msg):
        """
        Callback function for meta-command messages to this module.
        """
        
        self.lock.acquire()
        self.meta_command = msg
        self.lock.release()
    
    def state_callback(self, msg, args):
        """
        Callback function for state messages to this module. Capable of
        handling messages from multiple state topics simultaneously.
        
        See ROS Answers question #407730: 'ROS Subscribe to multiple
        topics with single function'
        """
        
        self.lock.acquire()
        key = args
        self.state[key] = msg
        self.lock.release()
    
    def run_once(self):
        """
        Run the components of the module in the proper order once.
        """
        
        start_t = rospy.Time.now()
        
        # Handle any meta-command received
        if self.meta_command is not None:
            self.handle_meta_command()
        
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
        
        # If no new directives, run activity manager
        if arb_directive is None:
            am_output = self.activity_manager.run(None, self.response)
            am_directives = am_output[0]
            am_responses = am_output[1]

            # Respond to commanded modules (if necessary)
            if am_directives is not None:
                self.publish_multiple_messages(am_directives)
        
        # If new directive from arbitration preempt activity manager
        else:
            am_directive = None
            am_responses = None
        
        # Control loop
        finished = False
        while not finished:
            
            # Run Control
            ctrl_output = self.control.run(arb_directive, am_responses, self.state)
            ctrl_directive = ctrl_output[0]
            ctrl_response = ctrl_output[1]
            
            # Handle new control directive output
            if ctrl_directive is not None:
                am_output = self.activity_manager.run(ctrl_directive, None)
                am_directives = am_output[0]
                am_responses = am_output[1]
                
                # Issue commands to modules
                if am_directives is not None:
                    self.publish_multiple_messages(am_directives)
                    finished = True
                
                # Finish if no new responses either
                elif am_responses is None:
                    finished = True
            
            # Handle new control response
            elif ctrl_response is not None:
                arb_output = self.arbitration.run(None, ctrl_response)
                arb_directive = arb_output[0]
                arb_response = arb_output[1]
                
                # Response to comanding module (if necessary)
                if arb_response is not None:
                    self.publish_message(arb_response)
                    finished = True
            
            # Otherwise finish current loop
            else:
                finished = True

        # Publish status message
        end_t = rospy.Time.now()
        self.publish_status_message(start_t, end_t)
        
        # Purge command and response callbacks for next loop
        self.command = None
        self.response = None
        self.meta_command = None
        
    def run(self):
        """
        Keep looping through the module while rospy is running.
        """
        
        # Main loop
        rospy.loginfo("'{}' node is running...".format(self.name))
        while not rospy.is_shutdown():
            self.run_once()
            self.rate.sleep()
            
    def handle_meta_command(self):
        """
        Execute 'meta-commands' provided from the architecture operator.
        
        TODO: Test
        """
        
        # Flush stored directives in arbitration
        if self.meta_command == "flush":
            self.arbitration.flush()
        
        # Reset the modules internal state
        elif self.meta_command == "reset":
            self.arbitration.reset()
            self.control.reset()
            self.activity_manager.reset()
        
        # TODO: Add more meta commands as necessary here
        #elif self.meta_command == ...:
    
    def check_for_completion(self):
        """
        Handling function for tactics when module is not recieving
        responses to its outputs (typically because its using an 
        interface). Creates a response message which elicits the correct
        response from the module.
        """
        
        # Get relevant information from arbitration and control component
        completion = self.control.tactic.completion
        fail_msg = self.control.tactic.fail_msg
        dir_id = self.control.cur_id
        
        # Create appropriate response message for situation
        if completion == "in progress":
            pass
        elif completion == "complete": #TODO: go to standby?
            self.response = create_response_obj(dir_id, "", self.name,
                                                "success", fail_msg, None, "")
        elif completion == "failed": #TODO: go to standby?
            self.response = create_response_obj(dir_id, "", self.name,
                                                "failure", fail_msg, None, "")
    
    def cleanup(self):
        """
        Things to do when shutdown occurs.
        """
        
        # Log shutdown of module
        rospy.sleep(1)
        rospy.loginfo("'{}' node is shutting down".format(self.name))
