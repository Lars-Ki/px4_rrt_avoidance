#!/usr/bin/env python
import rospy
import mavros_msgs
from mavros_msgs.msg import Trajectory
from mavros_msgs.msg import CompanionProcessStatus
from mavros_msgs.msg import State
import datetime
import time

class FlightMode():
    def __init__(self, flight_mode_status):
        self.flight_mode_status = flight_mode_status

def check_flight_mode(first_time):
    if (first_time == True):
        time_out = 100
    else:
        time_out = 3
    status = rospy.wait_for_message("mavros/state", mavros_msgs.msg.State, timeout= time_out )
    drone_1.flight_mode_status = status.system_status
    

def check_local_planner(first_time):
    if (first_time == True):
        time_out = 100          #first time tricky ..... better wait for a little bit  longer
    else:
        time_out = 3

    rospy.wait_for_message("/mavros/trajectory/generated", mavros_msgs.msg.Trajectory, timeout= time_out)
    give_signal()

def give_signal():
    status_msg = CompanionProcessStatus()
    status_msg.state = 4
    status_msg.component = 196
    time_stamp = int(rospy.get_rostime().to_sec())
    status_msg.header.stamp.secs = time_stamp
    
    print(status_msg)
    pub = rospy.Publisher('/mavros/companion_process/status', mavros_msgs.msg.CompanionProcessStatus, queue_size=5)
    pub.publish(status_msg)


if __name__ == "__main__":
    rospy.init_node('companion_comp_active')
    drone_1 = FlightMode(2837)
    check_flight_mode(True)
    '''
    check_local_planner(True)
    '''
    #check_flight_mode(True)
    while not rospy.is_shutdown():
        while True == True:
            '''
            check_local_planner(False)
            '''
            check_flight_mode(False)
            give_signal()

        rospy.spin()