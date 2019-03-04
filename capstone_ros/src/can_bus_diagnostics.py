#!/usr/bin/env python
# ****************************************************************************
# CAPSTONE G03, 2018-2019
#
# Repository:
#   Github: https://github.com/rahmant3/capstoneROS2018
#
# Description:
#   The ROS node in charge of interpreting the CAN messages and POSTing the
#   results to diagnostic ROS topics.
#
# History:
#   2018-12-21 by Tamkin Rahman
#   - Created.
#   2018-12-22 by Tamkin Rahman
#   - Removed Apache license from newer revisions of this file
#   2018-03-02 by Tamkin Rahman
#   - Remove IP that shouldn't be public (i.e. proprietary CAN message protocols).
# ****************************************************************************

# ****************************************************************************
#   IMPORTS
# ****************************************************************************
import rospy

from capstone_ros.msg import can

from std_msgs.msg import String
from std_msgs.msg import Bool
from std_msgs.msg import Float64

# ****************************************************************************
#   CONSTANTS
# ****************************************************************************

# ROS Path constants
ROS_PATH_VEHICLE_SPEED = "vehicle_speed"
ROS_PATH_ACTIVE_PCODES = "active_pcodes"

ROS_PATH_APS_FAULT = "aps_fault"
ROS_PATH_ATS_FAULT = "ats_fault"
ROS_PATH_ETS_FAULT = "ets_fault"
ROS_PATH_TPS_FAULT = "tps_fault"
ROS_PATH_FUEL_PUMP_FAULT = "fuel_pump_fault"
ROS_PATH_INJECTOR_FAULT = "injector_fault"
ROS_PATH_ENGINE_FAN_FAULT = "engine_fan_fault"
ROS_PATH_ECM_FAULT = "ecm_fault"
ROS_PATH_CAN_FAULT = "can_fault"
ROS_PATH_LOW_OIL = "low_oil"
ROS_PATH_LOW_BATTERY = "low_batt"
ROS_PATH_HI_BATTERY = "high_batt"
ROS_PATH_OTHER_FAULT = "other_fault"

ROS_PUBLISH_PATHS_TO_TYPE = {
    ROS_PATH_VEHICLE_SPEED: Float64,
    ROS_PATH_ACTIVE_PCODES: String,

    ROS_PATH_APS_FAULT: Bool,
    ROS_PATH_ATS_FAULT: Bool,
    ROS_PATH_APS_FAULT: Bool,
    ROS_PATH_ATS_FAULT: Bool,
    ROS_PATH_ETS_FAULT: Bool,
    ROS_PATH_TPS_FAULT: Bool,
    ROS_PATH_FUEL_PUMP_FAULT: Bool,
    ROS_PATH_INJECTOR_FAULT: Bool,
    ROS_PATH_ENGINE_FAN_FAULT: Bool,
    ROS_PATH_ECM_FAULT: Bool,
    ROS_PATH_CAN_FAULT: Bool,
    ROS_PATH_LOW_OIL: Bool,
    ROS_PATH_LOW_BATTERY: Bool,
    ROS_PATH_HI_BATTERY: Bool,

    ROS_PATH_OTHER_FAULT: Bool,
}

ROS_PATH_CAN0_RX = "can0_rx"
MAX_QUEUE_SIZE = 10

# Fault Type Enumerations
FAULT_TYPE_AIR_PRESS_SENSOR = 0
FAULT_TYPE_AIR_TEMP_SENSOR = 1
FAULT_TYPE_ENGINE_TEMP_SENSOR = 2
FAULT_TYPE_TPS = 3                  # Throttle Position Sensor
FAULT_TYPE_FUEL_PUMP = 4
FAULT_TYPE_INJECTOR = 5
FAULT_TYPE_ENGINE_FAN = 6
FAULT_TYPE_ECM = 7
FAULT_TYPE_CAN = 8

LOW_OIL = 252
LOW_BATTERY = 253
HI_BATTERY = 254

FAULT_TYPE_OTHER = 255

# Refer to pg. 185 - 192 of the 2011 Can-Am Service Manual for PCODE descriptions. Note that

# the keys are BCD format.
PCODES_TO_FAULT_TYPE = {
    0x0106: FAULT_TYPE_AIR_PRESS_SENSOR,
    0x0107: FAULT_TYPE_AIR_PRESS_SENSOR,
    0x0108: FAULT_TYPE_AIR_PRESS_SENSOR,
    0x0111: FAULT_TYPE_AIR_TEMP_SENSOR,
    0x0112: FAULT_TYPE_AIR_TEMP_SENSOR,
    0x0113: FAULT_TYPE_AIR_TEMP_SENSOR,
    0x0116: FAULT_TYPE_ENGINE_TEMP_SENSOR,
    0x0117: FAULT_TYPE_ENGINE_TEMP_SENSOR,
    0x0118: FAULT_TYPE_ENGINE_TEMP_SENSOR,
    0x0122: FAULT_TYPE_TPS,
    0x0123: FAULT_TYPE_TPS,
    0x0231: FAULT_TYPE_FUEL_PUMP,
    0x0232: FAULT_TYPE_FUEL_PUMP,
    0x0261: FAULT_TYPE_INJECTOR,
    0x0262: FAULT_TYPE_INJECTOR,
    0x0264: FAULT_TYPE_INJECTOR,
    0x0265: FAULT_TYPE_INJECTOR,
    0x0336: FAULT_TYPE_OTHER,
    0x0337: FAULT_TYPE_OTHER,
    0x0339: FAULT_TYPE_OTHER,
    0x0344: FAULT_TYPE_OTHER,
    0x0351: FAULT_TYPE_INJECTOR,
    0x0352: FAULT_TYPE_INJECTOR,
    0x0480: FAULT_TYPE_ENGINE_FAN,
    0x0505: FAULT_TYPE_OTHER,
    0x0513: FAULT_TYPE_OTHER,
    0x0520: FAULT_TYPE_OTHER,
    0x0562: LOW_BATTERY,
    0x0563: HI_BATTERY,
    0x0600: FAULT_TYPE_CAN,
    0x0601: FAULT_TYPE_ECM,
    0x0602: FAULT_TYPE_ECM,
    0x0604: FAULT_TYPE_ECM,
    0x0605: FAULT_TYPE_ECM,
    0x0608: FAULT_TYPE_OTHER,
    0x0616: FAULT_TYPE_OTHER,
    0x0617: FAULT_TYPE_OTHER,
    0x0705: FAULT_TYPE_OTHER,
    0x1104: FAULT_TYPE_OTHER,
    0x1116: FAULT_TYPE_OTHER,
    0x1148: FAULT_TYPE_OTHER,
    0x1202: FAULT_TYPE_OTHER,
    0x1203: FAULT_TYPE_OTHER,
    0x1520: LOW_OIL,
    0x1655: FAULT_TYPE_OTHER,
    0x1656: FAULT_TYPE_OTHER,
    0x1675: FAULT_TYPE_OTHER,
    0x1676: FAULT_TYPE_OTHER,
    0x1683: FAULT_TYPE_CAN,
    0x2119: FAULT_TYPE_ECM
}

FAULT_TYPE_TO_ROS_PATH = {
    FAULT_TYPE_AIR_PRESS_SENSOR: ROS_PATH_APS_FAULT,
    FAULT_TYPE_AIR_TEMP_SENSOR: ROS_PATH_ATS_FAULT,
    FAULT_TYPE_ENGINE_TEMP_SENSOR: ROS_PATH_ETS_FAULT,
    FAULT_TYPE_TPS: ROS_PATH_TPS_FAULT,
    FAULT_TYPE_FUEL_PUMP: ROS_PATH_FUEL_PUMP_FAULT,
    FAULT_TYPE_INJECTOR: ROS_PATH_INJECTOR_FAULT,
    FAULT_TYPE_ENGINE_FAN: ROS_PATH_ENGINE_FAN_FAULT,
    FAULT_TYPE_ECM: ROS_PATH_ECM_FAULT,
    FAULT_TYPE_CAN: ROS_PATH_CAN_FAULT,

    LOW_OIL: ROS_PATH_LOW_OIL,
    LOW_BATTERY: ROS_PATH_LOW_BATTERY,
    HI_BATTERY: ROS_PATH_HI_BATTERY,

    FAULT_TYPE_OTHER: ROS_PATH_OTHER_FAULT
}

SPEED_BIT_PER_KPH = 0.125

CAN_ID_PCODES = # Place CAN ID for PCODES msg here.
CAN_ID_SPEED = # Place CAN ID for Speed msg here.

# ****************************************************************************
#   GLOBALS
# ****************************************************************************
fault_publishers = {}

pcodes_mgr = None
vehicle_speed_kph = 0.0

# ****************************************************************************
#   CLASSES
# ****************************************************************************
class PCodesManager:

    def __init__(self):
        self.pcodes_list = []
        self.fault_states = {}

        for key in FAULT_TYPE_TO_ROS_PATH:
            self.fault_states[key] = False

    def set_pcode_active(self, pcode):
        ''' Used to notify this object that a PCODE has become active.

            Parameters:
                pcode: The active PCODE in BCD format.

            Returns:
                True, if this PCODE has been set to active. False if this
                PCODE was already marked active.
        '''
        rc = False

        if ((pcode not in self.pcodes_list) and (pcode in PCODES_TO_FAULT_TYPE)):
            self.pcodes_list.append(pcode)
            self.fault_states[PCODES_TO_FAULT_TYPE[pcode]] = True

            rc = True

        return rc

    def get_fault_state(self, fault):
        ''' Given a fault type, return whether that fault type is active.'''

        rc = False

        if (fault in self.fault_states):
            rc = self.fault_states[fault]

        return rc

    def get_active_pcodes_str(self):
        ''' Return a comma separated list of the active PCODES.'''
        self.pcodes_list.sort()

        s = " ,".join("P{0:04x}".format(x) for x in self.pcodes_list)

        return s


# ****************************************************************************
#   FUNCTIONS
# ****************************************************************************
def ros_init():
    # Reference: http://docs.ros.org/api/rospy/html/rospy.topics.Subscriber-class.html
    #            http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28python%29

    global fault_publishers
    global pcodes_mgr
    global vehicle_speed_kph

    rospy.init_node('diagnostic_can_node', anonymous=True)

    for key in ROS_PUBLISH_PATHS_TO_TYPE:
        fault_publishers[key] = rospy.Publisher(key, ROS_PUBLISH_PATHS_TO_TYPE[key], queue_size=MAX_QUEUE_SIZE)

    rospy.Subscriber(ROS_PATH_CAN0_RX,
                     can,
                     can_rx,
                     None,
                     MAX_QUEUE_SIZE
                     )
    rate = rospy.Rate(1)

    while not rospy.is_shutdown():
        for key in FAULT_TYPE_TO_ROS_PATH:
            fault_publishers[FAULT_TYPE_TO_ROS_PATH[key]].publish(pcodes_mgr.get_fault_state(key))

        fault_publishers[ROS_PATH_ACTIVE_PCODES].publish(pcodes_mgr.get_active_pcodes_str())
        fault_publishers[ROS_PATH_VEHICLE_SPEED].publish(vehicle_speed_kph)
        
        rate.sleep()

# ****************************************************************************
def can_rx(data, cb_args=None):
    global pcodes_mgr
    global vehicle_speed_kph

    # NOTE: In Python2 under ROS API, an array of uint8 is of type string.
    if ((data.id == CAN_ID_SPEED) and (len(data.data) == 8)):
        # Parse CAN data.

    elif ((data.id == CAN_ID_PCODES) and (len(data.data) == 8)):
        # Parse CAN data.

# ****************************************************************************
#   ENTRY POINT
# ****************************************************************************
if __name__ == '__main__':
    pcodes_mgr = PCodesManager()
    try:
        ros_init()
    except rospy.ROSInterruptException:
        pass

