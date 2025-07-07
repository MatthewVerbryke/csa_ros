#!/usr/bin/env python3

"""
  CSA module timeline plotting source code.
  
  Copyright 2021-2025 University of Cincinnati
  All rights reserved. See LICENSE file at:
  https://github.com/MatthewVerbryke/csa_ros
  Additional copyright may be held by others, as reflected in the commit
  history.
"""


import csv
import sys
from tkinter import filedialog

import pandas as pd
import plotly.express as px
import rosbag


class CSATimelinePlotter(object):
    """
    Timeline plotting object class for CSA modules.
    
    TODO: Test
    """
    
    def __init__(self):
        
        # Initalize variables
        self.module_names = None
        self.time = []
        self.statuses = {}
        self.shutdown = False
        
        # Ask user for rosbag file
        bag_file = filedialog.askopenfilename(
            title="Open a rosbag",
            initial_dir="/",
            file_types=(("rosbag", "*.bag"), ("All files", "*.*")),
        )
        
        # Retrieve desired rosbag file
        self.bag = rosbag.Bag(bag_file)
        
        # Parse out module status data
        for topic, msg, t in self.bag.read_messages(topics=["/csa_status"]):
            
            # Retrieve module indexes and prepare data storage dict
            if self.module_names is None:
                i = 0
                self.module_names = {}
                for status in msg[0]:
                    self.module_idx.update({status.name: i})
                    self.statuses.update({status.name: []})
                    i += 1
                
                # Retrieve overall list of module file name
                self.module_names = self.module_idx.keys()
            
            # Store time as float
            time = self.get_float_time(t)
            self.time.append(time)
            
            # Store status messages with relevant modules
            for key,value in self.module_idx.items():
                self.statuses[key].append(self.msg[value])
        
        # Run main loop
        self.main()
    
    def get_float_time(self, stamp):
        """
        Convert a stamp style representation of time (sec, nsec) into a
        floating-point representation.
        """
        
        # Convert nsec from nsecs (int) to seconds (float)
        float_nsecs = float(stamp.nsecs)*0.000000001
        
        # Add the two components together
        float_secs = float(stamp.secs) + float_nsecs
        
        return float_secs
    
    def convert_data_to_events(self, data_type):
        """
        Convert status data to pandas 'DataFrame' events, detecting mode
        start and end times for the selected mode data type (e.g. tactic,
        current directive, etc.) 
        """
        
        events_list = []
        modes = []
        
        # Retrieve loop start time and mode for each time step
        for key,value in self.statuses.items():
            start_t = []
            mode = []
            for msg in value:
                start_t.append(msg.values["start_time"])
                mode.append(msg.values[data_type])
                
                # Keep list of mode names
                if msg.values[data_type] not in modes:
                    modes.append(msg.values[data_type])
            
            # Start with first mode information
            cur_event = mode[0]
            cur_start = start_t[0]
            cur_end = None
            
            # Detect end of current mode and store as event dict
            for i in range(1, len(mode)):
                if mode[i] != cur_event:
                    cur_end = start_t[i]
                    event = dict(stack=key, start=cur_start, end=cur_end,
                                 task=cur_event)
                    events_list.append(event)
                    
                    # Start new event for next mode
                    cur_event = mode[i]
                    cur_start = start_t[i]
                    cur_end = None
            
            # Catch mode ending not ending within data set
            if cur_end is None:
                if cur_start != start_t[-1]:
                    cur_end = start_t[-1]
                    event = dict(stack=key, start=cur_start, end=cur_end,
                                 task=cur_event)
                    events_list.append(event)
                else:
                    pass
        
        # Store full events list as pandas DataFrame object
        events = pd.DataFrame(events_list)
        
        return modes, events
    
    def plot_basic_timeline(self, case):
        """
        Plot a timeline with only one mode (i.e. tactics, current 
        directive) presented per CSA module.
        """
        
        # Determine module mode to be displayed
        if case == 1:
            data_type = "tactic"
        elif case == 2:
            data_type = "directive"
            
        # Get data as event data frame
        modes, events = self.convert_data_to_events(data_type)
        
        # TODO: allow user to reorder module names and tasks in plot?
        
        # Plot timeline
        fig = px.timeline(
            events,
            x_start="start",
            x_end="end",
            y="stack",
            color="task",
            #color_discrete_map=...
            height=500,
            width=1200,
            #catagory_orders=...
            )
        
        fig.show()
    
    def main(self):
        """
        Main program loop with ability to perform multiple actions 
        without a program restart.
        """
        
        shutdown = False
        
        # Get desired action from user
        while not shutdown:
            print("ACTIONS:")
            print(" (0) Exit")
            print(" (1) Plot tactics timeline")
            print(" (2) Plot directives timeline")
            case = int(input("Enter case: "))
            
            # Exit command
            if case == 0:
                exit()
            
            # Plot basic timeline
            elif case == 1 or case == 2:
                self.plot_basic_timeline(case)


if __name__ == "__main__":
    CSATimelinePlotter()
