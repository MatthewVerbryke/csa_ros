#!/usr/bin/env python

"""
  Test code for CSA Activity Manager component.
  
  Copyright 2023 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/csa_ros
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


import copy

import rospy

from csa_module.activity_manager import ActivityManagerComponent
from csa_module.activity_manager_algorithm import ActivityManagerAlgorithm
from csa_msgs.msg import Directive, Response


class TestAMAlgorithm(ActivityManagerAlgorithm):
    
    def __init__(self):
        expect_resp = True
        super().__init__(expect_resp)
        self.case = 2
               
    def execute_activity(self, directive):
        if directive.name == "state_2":
            directive_out = [directive]
            self.outs = 1
        elif directive.name == "state_3":
            direct_1 = copy.deepcopy(directive)
            direct_1.id = 3
            direct_2 = copy.deepcopy(directive)
            direct_2.id = 4
            directive_out = [direct_1, direct_2]
            self.outs = 2
        
        return directive_out, True, ""
        
    def process_response(self, response):
        if self.outs == 1:
            mode_out = response.status
            params_out = response.params
        else:
            mode_out = "continue"
            params_out = None
            self.outs -= 1
        
        return mode_out, params_out

class TestActivityManager(object):
    """
    Class object for testing CSA Activity Manager component with a ros 
    node.
    
    NOTE: requires separate roscore to be running
    """
    
    def __init__(self):
        
        # Initialize node
        rospy.init_node("am_test_node", anonymous=True)
        
        # Setup necessary inputs
        name = "test_arbitration"
        am_algorithm = TestAMAlgorithm()
        
        # Initialize component
        self.am = ActivityManagerComponent(name, am_algorithm)
        
    def test_nominal_operation_no_response(self):
        """
        Test the component's nominal operation pattern.
        """
        
        print("")
        print("TEST 1 - NOMINAL OPERATION W/O RESPONSE")
        print("-----------------------------------------------------")
        
        # Set flag for response
        self.am.expect_resp = False
        
        # Standing by with no new directive
        output = self.am.run(None, None)
        self.print_status(output)
        
        # New directive from standby
        direct_1 = Directive()
        direct_1.name = "state_2"
        direct_1.id = 1
        output = self.am.run(direct_1, None)
        self.print_status(output)
    
        # Ensure no directives remain
        print("'self.directive':")
        print(self.am.directive)
        print("")
    
        # Running no new response
        output = self.am.run(None, None)
        self.print_status(output)

    def test_nominal_operation_response(self):
        """
        Test the component's nominal operation pattern.
        """
        
        print("")
        print("TEST 2 - NOMINAL OPERATION W/ RESPONSE")
        print("-----------------------------------------------------")
        
        # Set flag for response
        self.am.expect_resp = True
        
        # Standing by with no new directive
        output = self.am.run(None, None)
        self.print_status(output)
        
        # New directive from standby
        direct_1 = Directive()
        direct_1.name = "state_2"
        direct_1.id = 2
        output = self.am.run(direct_1, None)
        self.print_status(output)
    
        # Running no new response
        output = self.am.run(None, None)
        self.print_status(output)
        
        # Running got response
        resp_1 = Response()
        resp_1.id = 2
        resp_1.status = "success"
        output = self.am.run(None, resp_1)
        self.print_status(output)
        
        # Ensure no directives remain
        print("'self.directive':")
        print(self.am.directive)
        print("executing?:")
        print(self.am.executing)
        print("")
        
    def test_nominal_operation_multioutput(self):
        """
        Test the component's nominal operation pattern with multple
        outputs.
        """
        
        print("")
        print("TEST 3 - NOMINAL OPERATION W/ MULTIPLE OUTPUT DIRECTIVES")
        print("-----------------------------------------------------")
        
        # Standing by with no new directive
        output = self.am.run(None, None)
        self.print_status(output)
        
        # New directive from standby
        direct_1 = Directive()
        direct_1.name = "state_3"
        direct_1.id = 3
        output = self.am.run(direct_1, None)
        self.print_status(output)
        
        # Running no new response
        output = self.am.run(None, None)
        self.print_status(output)
        
        # Running got response
        resp_1 = Response()
        resp_1.id = 3
        resp_1.status = "success"
        output = self.am.run(None, resp_1)
        self.print_status(output)
        
        # Running no new response
        output = self.am.run(None, None)
        self.print_status(output)
        
        # Running got second response
        resp_2 = Response()
        resp_2.id = 4
        resp_2.status = "success"
        output = self.am.run(None, resp_2)
        self.print_status(output)
        
        # Ensure no directives remain
        print("'self.directive':")
        print(self.am.directive)
        print("executing?:")
        print(self.am.executing)
        print("")
    
    def test_failure(self):
        """
        Test component response to received failure response on the
        current directive(s).
        """
        
        print("")
        print("TEST 4 - HANDLE FAILED DIRECTIVE")
        print("-----------------------------------------------------")
    
        # Standing by with no new directive
        output = self.am.run(None, None)
        self.print_status(output)
        
        # New directive from standby
        direct_1 = Directive()
        direct_1.name = "state_2"
        direct_1.id = 4
        output = self.am.run(direct_1, None)
        self.print_status(output)
        
        # Running no new response
        output = self.am.run(None, None)
        self.print_status(output)
        
        # Running got response
        resp_1 = Response()
        resp_1.id = 5
        resp_1.status = "failure"
        resp_1.reject_msg = "User commanded failure"
        output = self.am.run(None, resp_1)
        self.print_status(output)
        
        # Ensure no directives remain
        print("'self.directive':")
        print(self.am.directive)
        print("executing?:")
        print(self.am.executing)
        print("")
        
    def print_status(self, output):
        """
        Print cleaned up outputs of current module.
        """
        
        print("Current ID:")
        print(self.am.cur_id)
        print("Directive Out:")
        print(output[0])
        print("Response Out:")
        print(output[1])
        print("")

if __name__ == "__main__":
    
    # Initialize component and test node
    am_test = TestActivityManager()
    
    # Run Test 1
    am_test.test_nominal_operation_no_response()
    
    # Run Test 2
    am_test.test_nominal_operation_response()
    
    # Run Test 3
    am_test.test_nominal_operation_multioutput()
    
    # Run Test 4
    am_test.test_failure()
