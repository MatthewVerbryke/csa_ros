#!/usr/bin/env python

"""
  Test code for CSA Control component.
  
  Copyright 2023-2024 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/csa_ros
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


import rospy
from std_msgs.msg import Float64

from csa_module.control import ControlComponent
from csa_msgs.msg import Directive, Response
from csa_module.tactics_algorithm import TacticsAlgorithm
from csa_module.tactic import Tactic


class FakeModel(object):
    
    def __init__(self):
        pass

class Tactic1(Tactic):
    
    def __init__(self, module_name, params, model):
        super().__init__(module_name, params, model)
        
    def run(self, state):
        directive = Directive()
        directive.name = "tactic_1"
        directive.priority = state.data + 10
        return True,[directive]

class Tactic2(Tactic):
    
    def __init__(self, module_name, params, model):
        super().__init__(module_name, params, model)
        
    def run(self, state):
        directive = Directive()
        directive.name = "tactic_2"
        directive.priority = state.data*10
        return True,[directive]

class FakeTacticAlgorithm(TacticsAlgorithm):   
    
    def __init__(self, tactic_list):
        super().__init__(tactic_list)
        
    def run(self, directive, state, model):
        if directive.name == "tactic_1":
            status = True
            tactic = Tactic1(self.module_name, None, model)
        elif directive.name == "tactic_2":
            status = True
            tactic = Tactic2(self.module_name, None, model)
        elif directive.name == "tactic_3":
            status = False
            tactic = None
            
        return status,tactic

class TestControlComponent(object):
    """
    Class object for testing CSA Control component with ros node.
    
    NOTE: requires separate roscore to be running
    """
    
    def __init__(self):
    
        # Initialize node
        rospy.init_node("ctrl_test_node", anonymous=True)
        
        # Setup necessary inputs
        name = "test_control"
        tactic_list = ["tactic_1", "tactic_2"]
        tactic_selection = FakeTacticAlgorithm(tactic_list)
        latency = 1.0
        tolerance = 0.1
        model = FakeModel()
        
        # Initialize component
        self.control = ControlComponent(name, tactic_selection, latency,
                                        tolerance, model)
        
    def test_nominal_operation(self):
        """
        Test the component's nominal operation pattern.
        """
        print("")
        print("TEST 1 - NOMINAL OPERATION")
        print("-----------------------------------------------------")
        
        # Pre-create state message
        state = Float64()
        state.data = 15
        
        # First run loop with default directive
        direct_1 = Directive()
        direct_1.name = "tactic_1"
        direct_1.id = -1
        output = self.control.run(direct_1, None, state)
        self.print_status(output)
        
        # Standing by with no new directive
        output = self.control.run(None, None, state)
        self.print_status(output)
        
        # New directive from standby
        direct_2 = Directive()
        direct_2.name = "tactic_2"
        direct_2.id = 1
        output = self.control.run(direct_2, None, state)
        self.print_status(output)
        
        # Running no new respose
        output = self.control.run(None, None, state)
        self.print_status(output)
        
        # Success response
        resp_1 = Response()
        resp_1.id = 1
        resp_1.status = "success"
        output = self.control.run(None, resp_1, state)
        self.print_status(output)
    
    def test_tactic_request_failure(self):
        """
        Test component handling failure to find tactic for current 
        directive.
        """
        
        print("")
        print("TEST 2 - FAILURE TO FIND TACTIC")
        print("-----------------------------------------------------")
        
        # Pre-create state message
        state = Float64()
        state.data = 15
        
        # First run loop with default directive
        direct_1 = Directive()
        direct_1.name = "tactic_1"
        direct_1.id = -1
        output = self.control.run(direct_1, None, state)
        self.print_status(output)
        
        # Standing by with no new directive
        output = self.control.run(None, None, state)
        self.print_status(output)
        
        # New directive from standby
        direct_2 = Directive()
        direct_2.name = "tactic_3"
        direct_2.id = 2
        output = self.control.run(direct_2, None, state)
        self.print_status(output)
        
    def test_new_directive(self):
        """
        Test component handling of new directive while exectuing.
        """
        
        print("")
        print("TEST 3 - NEW DIRECTIVE DURING EXECUTION")
        print("-----------------------------------------------------")
        
        # Pre-create state message
        state = Float64()
        state.data = 15
        
        # First run loop with default directive
        direct_1 = Directive()
        direct_1.name = "tactic_1"
        direct_1.id = -1
        output = self.control.run(direct_1, None, state)
        self.print_status(output)
        
        # Standing by with no new directive
        output = self.control.run(None, None, state)
        self.print_status(output)
        
        # New directive from standby
        direct_2 = Directive()
        direct_2.name = "tactic_2"
        direct_2.id = 3
        output = self.control.run(direct_2, None, state)
        self.print_status(output)
        
        # Running no new respose
        output = self.control.run(None, None, state)
        self.print_status(output)
        
        # New directive from standby
        direct_3 = Directive()
        direct_3.name = "tactic_2"
        direct_3.id = 4
        output = self.control.run(direct_3, None, state)
        self.print_status(output)
        
        # Running no new respose
        output = self.control.run(None, None, state)
        self.print_status(output)
        
        # Success response
        resp_1 = Response()
        resp_1.id = 4
        resp_1.status = "success"
        output = self.control.run(None, resp_1, state)
        self.print_status(output)
        
    def test_directive_failure(self):
        """
        Test component response to received failure response on the
        current directive.
        """
        
        print("")
        print("TEST 4 - HANDLE FAILED DIRECTIVE")
        print("-----------------------------------------------------")
        
        # Pre-create state message
        state = Float64()
        state.data = 15
        
        # First run loop with default directive
        direct_1 = Directive()
        direct_1.name = "tactic_1"
        direct_1.id = -1
        output = self.control.run(direct_1, None, state)
        self.print_status(output)
        
        # Standing by with no new directive
        output = self.control.run(None, None, state)
        self.print_status(output)
        
        # New directive from standby
        direct_2 = Directive()
        direct_2.name = "tactic_2"
        direct_2.id = 5
        output = self.control.run(direct_2, None, state)
        self.print_status(output)
        
        # Running no new respose
        output = self.control.run(None, None, state)
        self.print_status(output)
        
        # Failure response
        resp_1 = Response()
        resp_1.id = 5
        resp_1.status = "failure"
        resp_1.reject_msg = "User created test failure"
        output = self.control.run(None, resp_1, state)
        self.print_status(output)
        
    def print_status(self, output):
        """
        Print cleaned up outputs of current module.
        """
        
        print("Current ID:")
        print(self.control.cur_id)
        print("Control Directive Out:")
        print(output[0])
        print("Response Out:")
        print(output[1])
        print("")


if __name__ == "__main__":
    
    # Initialize component and test node
    ctrl_test = TestControlComponent()
    
    # Run Test 1
    ctrl_test.test_nominal_operation()
    
    # Run Test 2
    ctrl_test.test_tactic_request_failure()
    
    # Run Test 3
    ctrl_test.test_new_directive()
    
    # Run Test 4
    ctrl_test.test_directive_failure()
