#!/usr/bin/env python3

"""
  CSA module monitor node source code
  
  Copyright 2025 University of Cincinnati
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


class CSAMonitor(object):
    """
    A architecture wide CSA-module monitor object.
    """
    
    def __init__(self):
        
        # Get home directory
        self.home_dir = os.getcwd()
        
        # Initialize rospy node
        rospy.init_node("csa_monitor")
        
        # Setup cleanup function
        rospy.on_shutdown(self.cleanup)
        
        # Get lock
        self.lock = threading.Lock()
        
        # Get module parameters
        self.robot = rospy.get_param("~robot", "")
        rate = rospy.get_param("~rate", 30.0)
        self.rate = rospy.Rate(rate)
        self.module_names = rospy.get_param("~module_names", [])
        
        # Signal initialization
        rospy.loginfo("CSA monitor node initialized")
        
        # Initialize variables
        self.status = {}
        
        # Setup overall publisher
        self.publisher = rospy.Publisher("rse_status", DiagnosticArray,
                                         queue_size=1)
        
        # Initialize subscribers objects
        self.subscribers = {}
        for name in self.module_name:
            self.setup_subscriber(name)
    
    def setup_subscriber(self, module_name)
        """
        Setup a rospy subscriber for a particular CSA module
        """
        
        # Setup topic name
        sub_topic = module_name + "/status"
        
        # Create subscriber
        sub = rospy.Subscriber(name=sub_topic,
                               data_class=DiagnosticStatus,
                               callback=self.status_callback,
                               callback_args=module_name)
        
        # Package into dictionary
        self.subscribers.update({module_name: sub})
        self.status.update({module_name: None})
    
    def status_callback(self, msg, args):
        """
        Callback function for module status messages; Capable of 
        handling messages from multiple state topics simultaneously.
        
        See ROS Answers question #407730: 'ROS Subscribe to multiple
        topics with single function'
        """
        
        self.lock.acquire()
        key = args
        self.status[key] = msg
        self.lock.release()
        
    def run(self):
        """
        Keep looping through the module while rospy is running.
        
        # TODO: Add more functionality as needed
        """
        
        # Main loop
        rospy.loginfo("CSA monitor node is running...".format(self.name))
        while not rospy.is_shutdown():
            self.publisher.publish(self.status)
            self.rate.sleep()
    
    def cleanup(self):
        """
        Things to do when shutdown occurs.
        """
        
        # Log shutdown of module
        rospy.sleep(1)
        rospy.loginfo("CSA monitor node shutting down")
