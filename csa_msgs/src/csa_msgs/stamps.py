#!/usr/bin/env python3

"""

"""


import rospy


def get_stamp(float_time):
    """
    Convert a floating-point representation of time in seconds into a 
    ros_msgs 'stamp' style representation.
    """

    # Convert to time in seconds (float) to time in nanoseconds (int)
    nsec = rospy.Time.from_sec(float_time)

    # Get the number of 'whole' seconds
    sec = int(float_time)
    
    return sec, nsec.nsecs
    
def get_float_time(stamp):
    """
    Convert a stamp style representation of time (sec, nsec) into a floating-
    point representation.
    """
    
    # Convert nsec from nsecs (int) to seconds (float)
    float_nsecs = float(stamp.nsecs)*0.000000001
    
    # Add the two components together
    float_secs = float(stamp.secs) + float_nsecs

    return float_secs
