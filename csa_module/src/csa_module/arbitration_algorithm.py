#!/usr/bin/env python3

"""
  Generic CSA arbitration algorithm base class source code.
  
  Copyright 2023 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/csa_ros
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


from csa_common.directed_graph import DirectedGraph


class ArbitrationAlgorithm(object):
    """
    Base class object for arbitration algorithms.
    """
    
    def __init__(self, allowed_dirs, allowed_list):
        
        # Store parameters
        self.allowed_dirs = allowed_dirs
        self.dir_graph = DirectedGraph(allowed_dirs, allowed_list)
    
    def run(self, cur_directive, directives):
        """
        Run the arbitration algorithm (needs to be filled end by user).
        """
        pass
