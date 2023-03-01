#!/usr/bin/env python3

"""
  Newest arbitration function source code.
  
  Copyright 2023 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/csa_ros
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


from csa_module.arbitration_algorithm import ArbitrationAlgorithm
from csa_msgs.stamps import get_float_time


class NewestArbitration(ArbitrationAlgorithm):
    """
    Arbitration function to choose the newest directive when arbitrating
    over several directives.
    """
    
    def __init__(self, arb_list, adjacencies):
        super().__init__(arb_list, adjacencies)
        
    def run(self, cur_directive, directives):
        """
        Run the arbitration algorithm.
        """
        
        ids = []
        times = []
        
        # Get out times corrosponding to keys
        for key,value in directives.items():
            ids.append(key)
            times.append(get_float_time(value.header.stamp))
        
        # Determine newest keys and ids
        newest_id = times.index(max(times))
        newest_key = ids[newest_id]
        
        # Get out newest directive
        directive_out = directives[newest_key]
        
        return directive_out


if __name__ == "__main__":
    
    from csa_msgs.directive import Directive
    
    # Initialize test arbitration function
    allowed_list = ["dir1", "dir2", "dir3"]
    adjacency = [[1, 1, 1],
                 [1, 1, 1],
                 [1, 1, 1]]
    test_arb = NewestArbitration(allowed_list, adjacency)

    # First Directive
    dir_1 = Directive()
    dir_1.name = "dir1"
    dir_1.header.stamp.secs = 1.500
    dir_1.header.stamp.nsecs = 0.000
    
    # Second directive
    dir_2 = Directive()
    dir_2.name = "dir2"
    dir_2.header.stamp.secs = 1.600
    dir_2.header.stamp.nsecs = 0.000
    
    # Test functionality
    directives = {1: dir_1, 2: dir_2}
    directive_out = test_arb.run(dir_1, directives)
    print(directive_out)
