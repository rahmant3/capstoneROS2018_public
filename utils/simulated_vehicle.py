#!/usr/bin/env python
# ****************************************************************************
# CAPSTONE G03, 2018-2019
#
# License:
#   This software is available under the Apache-2.0 license.
#
# Repository:
#   Github: https://github.com/rahmant3/capstoneROS2018
#
# Description:
#   Simulated vehicle application for publishing to diagnostic topics for the
#   controller applications.
#
# History:
#   2018-11-14 by Tamkin Rahman
#   - Created.
#   2018-12-21 by Tamkin Rahman
#   - Update to simulate newly selected diagnostic parameters.
# ****************************************************************************

# ****************************************************************************
#   IMPORTS
# ****************************************************************************
import rospy
from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Float64

# ****************************************************************************
#   CONSTANTS
# ****************************************************************************
MAX_QUEUE_SIZE = 100

# 2-D Array containing parameters for initializing the ROS subscribers, where
# each entry is like so:
#   1. The ROS subscriber path.
#   2. The ROS type to get.
#   3. The ROS param for the callback.
ROS_PARAMS = { 
                "active_pcodes": [String, "P0616, P0617"],
                "vehicle_speed": [Float64, 0.0],
                "low_batt": [Bool, False],
                "high_batt": [Bool, False],
                "low_oil": [Bool, False],
                "aps_fault": [Bool, False],
                "ats_fault": [Bool, False],
                "ets_fault": [Bool, False],
                "tps_fault": [Bool, False],
                "fuel_pump_fault": [Bool, True],
                "injector_fault": [Bool, False],
                "engine_fan_fault": [Bool, True],
                "ecm_fault": [Bool, False],
                "can_fault": [Bool, False],
                "other_fault": [Bool, False]
            }
               

# ****************************************************************************
#   VARIABLES
# ****************************************************************************
post_params = {}

# ****************************************************************************
#   FUNCTIONS
# ****************************************************************************
def talker():

    # Reference: http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29
    pubs = {}

    for key in ROS_PARAMS:
        pubs[key] = rospy.Publisher(key, ROS_PARAMS[key][0], queue_size=10)

    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        for key in ROS_PARAMS:
            rospy.loginfo("Sending {0} on {1}".format(ROS_PARAMS[key][1], key))
            pubs[key].publish(ROS_PARAMS[key][1])

        rate.sleep()


# ****************************************************************************
#   ENTRY POINT
# ****************************************************************************
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
