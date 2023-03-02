#!/usr/bin/env python3

"""
  Highest priority arbitration function source code.
  
  Copyright 2023 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/csa_ros
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


from csa_module.arbitration_algorithm import ArbitrationAlgorithm


class PriorityArbitration(ArbitrationAlgorithm):
    """
    Arbitration function to choose the highest priority directive when
    arbitrating over several directives.
    """
    
    def __init__(self, arb_list, arb_edges):
        super().__init__(arb_list, arb_edges)
    
    def run(self, cur_directive, directives):
        """
        Run the arbitration algorithm.
        """
        
        ids = []
        priorities = []
        
        # Get out priorities corrosponding to keys
        for key,value in directives.items():
            ids.append(key)
            priorities.append(value.priority)
        
        # Determine highest priority directive 
        top_index = priorities.index(max(priorities))
        top_key = ids[top_index]
        
        # Get out newest directive
        directive_out = directives[top_key]
        
        return directive_out
        

if __name__ == "__main__":
    
    from csa_msgs.directive import Directive
    
    # Initialize test arbitration function
    allowed_list = ["dir1", "dir2", "dir3"]
    adjacency = [[1, 1, 1],
                 [1, 1, 1],
                 [1, 1, 1]]
    test_arb = PriorityArbitration(allowed_list, adjacency)

    # First directive
    dir_1 = Directive()
    dir_1.name = "dir1"
    dir_1.priority = 0
    
    # Second directive
    dir_2 = Directive()
    dir_2.name = "dir2"
    dir_2.priority = 1
    
    # Test functionality
    directives = {1: dir_1, 2: dir_2}
    directive_out = test_arb.run(dir_1, directives)
    print(directive_out)
