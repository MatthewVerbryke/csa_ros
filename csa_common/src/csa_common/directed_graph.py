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
        
    def add_fully_connected_node(self, node):
        """
        Add a new node with fully connected to the other nodes, which
        represents adding a new directive type which can be called when
        starting while executing any other node.
        
        NOTE: make sure name of directive is added to end of allowed 
              directive list.
        """
        
        # Append node name to nodes list
        self.nodes.append(node)
        
        # Create new adjacency matrix one size bigger on each axis
        a_shape = self.a_matrix.shape
        new_a_matrix = np.ones((a_shape[0]+1, a_shape[1]+1), dtype=int)
        
        # Put old adjacency matrix in upper left of new matrix
        new_a_matrix[0:a_shape[0], 0:a_shape[0]] = self.a_matrix
        
        # Store the new matrix
        self.a_matrix = new_a_matrix
        
    def check_adjacency(self, start, end):
        """
        Return whether or not the selected nodes have a connecting edge.
        """
        
        # Find indices for each value
        i0 = self.nodes.index(start)
        i1 = self.nodes.index(end)
        
        # Get value out of adjacency matrix
        value = self.a_matrix[i0,i1]
        
        # Determine output
        if value == 1:
            allowed = True
        else:
            allowed = False
            
        return allowed
