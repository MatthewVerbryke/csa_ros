#!/usr/bin/env python3

"""
  Directed graph source code.
  
  Copyright 2023 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/csa_ros
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


import numpy as np


class DirectedGraph(object):
    """
    A generic, very-simple directed graph object.
    """
    
    def __init__(self, nodes, adjacencies):        
        
        # Store parameters
        self.nodes = nodes
        self.a_matrix = np.matrix(adjacencies)
    
    def check_adjacency(self, start, end):
        """
        Return whether or not the selected nodes have a connecting edge.
        """
        
        # Get value out of adjacency matrix
        value = self.a_matrix[start,end]
        
        # Determine output
        if value == 1:
            allowed = True
        else:
            allowed = False
            
        return allowed
