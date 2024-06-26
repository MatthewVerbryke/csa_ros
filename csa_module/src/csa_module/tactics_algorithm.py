#!/usr/bin/env python3

"""
  Generic CSA tactic selection algorithm base class source code.
  
  Copyright 2023-2024 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/csa_ros
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


from diagnostic_msgs.msg import KeyValue
import rospy


class TacticsAlgorithm(object):
    """
    Base class object for a tactic selection algorithm.
    """
    
    def __init__(self, tactic_list):
        
        # Store list of tactics which can be called
        self.tactic_list = tactic_list
        
        # TODO: add checks for this ^?
        
    def get_module_info(self, module_name, rate, latency):
        """
        Store important information about module, including its name,
        execution rate, and expected latency.
        """
        self.module_name = module_name
        self.rate = rate
        self.latency = latency
        
    def get_cur_id(self, directive):
        """
        Retrieve the current ID number from the directive, so various
        tactics have it when running.
        """
        id_kv = KeyValue()
        id_kv.key = "id"
        id_kv.value = str(directive.id)
        
        return id_kv
    
    def run(self, directive, state, model):
        """
        Run the tactics selection algorithm (needs to be filled end by
        user).
        """
        
        tactics = None
        status = False
        
        return success, tactics
