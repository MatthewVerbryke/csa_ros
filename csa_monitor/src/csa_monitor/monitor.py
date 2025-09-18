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

from csa_msgs.msg import Status, ModuleStatus
import rospy


class CSAMonitor(object):
    """
    A architecture wide CSA-module monitor object.
    
    TODO: Test
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
        self.print_topic = rospy.get_param("~print_node", None)
        
        # Signal initialization
        rospy.loginfo("CSA monitor node initialized")
        
        # Initialize variables
        self.status = {}
        self.last_print = None
        
        # Setup overall publisher
        self.publisher = rospy.Publisher("csa_status", Status,
                                         queue_size=1)
        
        # Initialize subscribers objects
        self.subscribers = {}
        for name in self.module_names:
            self.setup_subscriber(name)
            
        # Run monitor
        self.run()
    
    def setup_subscriber(self, module_name):
        """
        Setup a rospy subscriber for a particular CSA module
        """
        
        # Setup topic name
        sub_topic = module_name + "/status"
        
        # Create subscriber
        sub = rospy.Subscriber(name=sub_topic,
                               data_class=ModuleStatus,
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
    
    def publish_status_array(self):
        """
        Convert status dictionary into a DiagnosticArray type message
        and publish it.
        """
        
        # Construct Status message from module status info
        status_msg = Status()
        status_msg.stamp = rospy.Time.now()
        for key,value in self.status.items():
            status_msg.system_status.append(value)
            if key == self.print_topic:
                print(value)
                print("------------------------------------")
            
        # Publish status
        self.publisher.publish(status_msg)
    
    def print_node_status(self):
        """
        If a specific module output is specified, print it out if it has
        changed.
        """
        
        if self.last_print != self.status[self.print_topic]:
            print(self.status[self.print_topic])
    
    def run(self):
        """
        Keep looping through the module while rospy is running.
        
        TODO: Add more functionality as needed
        """
        
        # Main loop
        rospy.loginfo("CSA monitor node is running...")
        while not rospy.is_shutdown():
            self.publish_status_array()
            if self.print_topic is not None:
                self.print_node_status()
            self.rate.sleep()
    
    def cleanup(self):
        """
        Things to do when shutdown occurs.
        """
        
        # Log shutdown of module
        rospy.sleep(1)
        rospy.loginfo("CSA monitor node shutting down")


if __name__ == "__main__":
    CSAMonitor()
