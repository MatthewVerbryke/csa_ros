#!/usr/bin/env python

"""
  Test code for CSA Arbitration component.
  
  Copyright 2023 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/csa_ros
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


import rospy

from csa_module.arbitration import ArbitrationComponent
from csa_common.arb_newest import NewestArbitration
from csa_msgs.msg import Directive, Response


class TestArbitrationComponent(object):
    """
    Class object for testing CSA Arbitration component with ros node.
    
    NOTE: requires separate roscore to be running
    """
    
    def __init__(self):
    
        # Initialize node
        rospy.init_node("arb_test_node", anonymous=True)
        
        # Setup necessary inputs
        name = "test_arbitration"
        default_name = "state_1"
        max_directive = 2
        arb_algorithm = NewestArbitration(["state_1", "state_2", "state_3"],
                                          [[1,1,0], [1,0,1], [0,1,1]])
        
        # Initialize component
        self.arbitration = ArbitrationComponent(name, arb_algorithm,
                                                default_name, max_directive)
    
    def test_nominal_operation(self):
        """
        Test the component's nominal operation pattern.
        """
        print("")
        print("TEST 1 - NOMINAL OPERATION")
        print("-----------------------------------------------------")
        
        # First run loop
        output = self.arbitration.run(None, None)
        self.print_status(output)
        
        # Standing by with no directive
        output = self.arbitration.run(None, None)
        self.print_status(output)
        
        # New directive from standby
        direct_1 = Directive()
        direct_1.name = "state_2"
        direct_1.id = 1
        output = self.arbitration.run(direct_1, None)
        self.print_status(output)
        
        # Waiting for response
        output = self.arbitration.run(None, None)
        self.print_status(output)
        
        # Success response
        resp_1 = Response()
        resp_1.status = "success"
        resp_1.id = 1
        output = self.arbitration.run(None, resp_1)
        self.print_status(output)
        
        # Ensure no directives remain
        print("'self.directives' list:")
        print(self.arbitration.directives)
        print("")
        
        # Standing by with no directive
        output = self.arbitration.run(None, None)
        self.print_status(output)
        
    def test_new_directives(self):
        """
        Test component handling of new directive while exectuing.
        """
        
        print("")
        print("TEST 2 - NEW DIRECTIVE DURING EXECUTION")
        print("-----------------------------------------------------")
        
        # Standing by with no directive
        output = self.arbitration.run(None, None)
        self.print_status(output)
        
        # New directive from standby
        direct_1 = Directive()
        direct_1.name = "state_2"
        direct_1.id = 2
        direct_1.header.stamp = rospy.Time.now()
        output = self.arbitration.run(direct_1, None)
        self.print_status(output)
        
        # Waiting for response
        output = self.arbitration.run(None, None)
        self.print_status(output)
        
        # New directive on-top of current one
        direct_2 = Directive()
        direct_2.name = "state_3"
        direct_2.id = 3
        direct_2.header.stamp = rospy.Time.now()
        output = self.arbitration.run(direct_2, None)
        self.print_status(output)
        
        # Reject new directive over limit
        direct_3 = Directive()
        direct_3.name = "state_2"
        direct_3.id = 4
        direct_3.header.stamp = rospy.Time.now()
        direct_3.source = "fake_module"
        output = self.arbitration.run(direct_3, None)
        self.print_status(output)
        
        # Success response
        resp_1 = Response()
        resp_1.status = "success"
        resp_1.id = 3
        output = self.arbitration.run(None, resp_1)
        self.print_status(output)
        
        # Waiting for response
        output = self.arbitration.run(None, None)
        self.print_status(output)
        
        # Success response 2
        resp_2 = Response()
        resp_2.status = "success"
        resp_2.id = 2
        output = self.arbitration.run(None, resp_2)
        self.print_status(output)
        
        # Ensure no directives remain
        print("'self.directives' list:")
        print(self.arbitration.directives)
        print("")
        
        # Standing by with no directive
        output = self.arbitration.run(None, None)
        self.print_status(output)
        
    def test_failure(self):
        """
        Test component response to received failure response on the
        current directive.
        """
        
        print("")
        print("TEST 3 - HANDLE FAILED DIRECTIVE")
        print("-----------------------------------------------------")
        
        # Standing by with no directive
        output = self.arbitration.run(None, None)
        self.print_status(output)
        
        # New directive from standby
        direct_1 = Directive()
        direct_1.name = "state_2"
        direct_1.id = 5
        direct_1.header.stamp = rospy.Time.now()
        output = self.arbitration.run(direct_1, None)
        self.print_status(output)
        
        # Waiting for response
        output = self.arbitration.run(None, None)
        self.print_status(output)
        
        # Failure response
        resp_1 = Response()
        resp_1.status = "failure"
        resp_1.id = 5
        resp_1.reject_msg = "User commanded failure"
        output = self.arbitration.run(None, resp_1)
        self.print_status(output)
        
        # Ensure no directives remain
        print("'self.directives' list:")
        print(self.arbitration.directives)
        print("")
        
        # Standing by with no directive
        output = self.arbitration.run(None, None)
        self.print_status(output)
        
    def print_status(self, output):
        """
        Print cleaned up outputs of current module.
        """
        
        print("Current ID:")
        print(self.arbitration.cur_id)
        print("Directive Out:")
        print(output[0])
        print("Response Out:")
        print(output[1])
        print("")


if __name__ == "__main__":
    
    # Initialize component and test node
    arb_test = TestArbitrationComponent()
    
    # Run Test 1
    arb_test.test_nominal_operation()
    
    # Run Test 2
    arb_test.test_new_directives()
    
    # Run Test 3
    arb_test.test_failure()
