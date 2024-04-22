#!/usr/bin/env python

"""
  Code for CSA module test commander
  
  Copyright 2023-2024 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/csa_ros
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


import threading

import rospy

from csa_msgs.msg import Directive, Response
from csa_msgs.directive import create_directive_msg
from csa_msgs.params import create_param_submsg


class ModuleTestCommander(object):
    """
    Module test commander node object. Mimics the function of a CSA 
    module to test acutal CSA modules, by sending out a predifined 
    directive, and listening to the response(s). 
    """
    
    def __init__(self, fake_name, stop_option, pub_topic, param_inputs,
                 dir_inputs):
        
        # Initialize rospy node
        rospy.init_node("module_commander")
        
        # Setup cleanup function
        rospy.on_shutdown(self.cleanup)
        
        # Get lock
        self.lock = threading.Lock()
        
        # Store module name
        self.name = "module_commander"
        
        # Get rospy rate object
        self.rate = rospy.Rate(30.0)
        
        # Store stop option
        self.stop_option = stop_option
        
        # Create and store directive object
        deadline = rospy.Time.now() + rospy.Duration(param_inputs[4])
        params = create_param_submsg(param_inputs[0], param_inputs[1],
                                     param_inputs[2], param_inputs[3],
                                     deadline)
        self.directive = create_directive_msg(dir_inputs[0], dir_inputs[1],
                                              dir_inputs[2], dir_inputs[3],
                                              dir_inputs[4], dir_inputs[5],
                                              dir_inputs[6], params,
                                              dir_inputs[7])
        
        # Setup Response subscriber
        resp_topic = fake_name + "/response"
        self.sub = rospy.Subscriber(resp_topic, Response, 
                                    self.response_callback)
        
        # Setup Directive publisher
        self.pub = rospy.Publisher(pub_topic, Directive, queue_size=1)
        
        # Create empty response holding variable
        self.response = None
        
        # Setup initial flag variables
        self.got_accept = False
        self.got_response = False
        
        # Start running execution loop
        self.run()
        
    def response_callback(self, msg):
        """
        Callback function for response messages to this module.
        """
        
        # Store incoming response message
        self.lock.acquire()
        self.response = msg
        self.lock.release()
    
    def send_until_accepted(self):
        """
        Send message until the module has recieved an acceptance
        response.
        """
        
        # Publish test directive
        self.pub.publish(self.directive)
        
        # Process any response
        if self.response is not None:
            if self.response.status == "accept":
                print("heard acceptance message")
                self.got_accept = True
            else:
                print("heard rejection message")
        else:
            pass
        
        # Reset response message
        self.response = None
    
    def run(self):
        """
        Keep looping through the module while rospy is running.
        """
        
        # Main loop
        rospy.loginfo("'%s' node is running...", self.name)
        while not self.got_response:
            if not self.got_accept:
                self.send_until_accepted()
            else:
                if self.stop_option:
                    rospy.loginfo("Stoping node due to stop option True")
                    break
                else:
                    if self.response is not None:
                        print("heard response!")
                        print(self.response)
                        rospy.loginfo("Result: '%s'", self.response.status)
                        exit()
            self.rate.sleep()
        
    def cleanup(self):
        """
        Things to do when shutdown occurs.
        """
        
        # Log shutdown of module
        rospy.sleep(1)
        rospy.loginfo("Shutting down '%s' node", self.name)
    
