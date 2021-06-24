#!/usr/bin/env python
import rospy
import mavros_msgs
from mavros_msgs.msg import Trajectory
from mavros_msgs.msg import CompanionProcessStatus
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandTOL 
from px4_rrt_avoidance.msg import DynamicObstacle
from px4_rrt_avoidance.msg import Path_coll
from px4_rrt_avoidance.msg import RRT_Star_Call
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import OccupancyGrid 
from local_planner import calc_angle
from debugging_functions import show_checked_path
from debugging_functions import visualize_path
import geometry_msgs
import numpy as np
import math
import time
import sys

def pubPosition(position):
    pub = rospy.Publisher('/dynamic_obstacle', DynamicObstacle, queue_size=1)
    msg = DynamicObstacle()
    msg.position.x = position[0]
    msg.position.y = position[1]
    msg.position.z = position[2]

    msg.DroneID = 2

    pub.publish(msg)

    return True




if __name__ == "__main__":
    rospy.init_node("test_dyn_obstacle", anonymous=False)

    x_pos = 15
    z_pos = 15

    y_pos = -14

    while not rospy.is_shutdown():
        if y_pos <= 14:
            y_pos += 1

            vec = [x_pos, y_pos, z_pos]
            pubPosition([x_pos, y_pos, z_pos])
            print(vec)
            rospy.sleep(0.5)

        else:
            y_pos = -14
            vec = [x_pos, y_pos, z_pos]
            pubPosition([x_pos, y_pos, z_pos])
            print(vec)
            rospy.sleep(0.5)

#    
    #rospy.spin()