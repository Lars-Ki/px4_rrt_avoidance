#!/usr/bin/env python
import rospy
import roslib
import tf
import sys
import os
import random
import math
import nav_msgs
import geometry_msgs
from nav_msgs.msg import OccupancyGrid 
import numpy as np
from mavros_msgs.msg import PositionTarget
from mavros_msgs.msg import Trajectory
from px4_rrt_avoidance.msg import Path
import mavros_msgs
import time



class Points:
    def __init__(self):
        self.x  = None
        self.y  = None
        self.z  = None
        self.yaw = None
        self.type_mask = None
        self.point_valid = None



class Trajectory:
    def __init__(self, desired , generated, time):
        self.desired = desired
        self.generated = generated
        self.time = time

    def show_msg(self):
        print("#############################################")
        print("Time: " + str(self.time))
        print("Desired: Point 2:")

        print("x: " + str(self.desired[0].x))
        print("y: " + str(self.desired[0].y))
        print("z: " + str(self.desired[0].z))
        print('type_mask: ' + str(self.desired[0].type_mask))
        print("  ------------------------------------------ ")

        print("generated: Point 1:")
        print("x: " + str(self.generated[0].x))
        print("y: " + str(self.generated[0].y))
        print("z: " + str(self.generated[0].z))
        print("yaw: " +str(self.generated[0].yaw))
        print("type_mask: " +str(self.generated[0].type_mask))
        print('..')
        print("generated: Point 2:")
        print("x: " + str(self.generated[1].x))
        print("y: " + str(self.generated[1].y))
        print("z: " + str(self.generated[1].z))
        print("yaw: " +str(self.generated[1].yaw))
        print("type_mask: " +str(self.generated[1].type_mask))
        print('Valid: ' + str(self.generated[1].point_valid))
        valid_values = self.generated[1].point_valid
        #print('[ '+ str(valid_values[0]) + ', ' + str(valid_values[1]) + ' , ' + str(valid_values[2]) + ', .. ]')
        print("###############################################")
        pass



def show_msg(msg):

    point_1 = trajectory.desired[0]
    point_1.x = msg.point_2.position.x
    point_1.y = msg.point_2.position.y
    point_1.z = msg.point_2.position.z
    point_1.type_mask = msg.point_2.type_mask

#    desired = []
#    desired.append(msg.point_2.position.x)
#    desired.append(msg.point_2.position.y)
#    desired.append(msg.point_2.position.z)
#    #print("setting")
#    trajectory.desired = desired

    time = msg.header.stamp.secs
    trajectory.time = time
    #print("/trajectory/desired Point 1: ")
    #print(msg.point_1.position)

    #print("Point 2:")
    #print(msg.point_2.position)
    #print("###################")

def show_generated(msg):

    point_1 = trajectory.generated[0]
    point_2 = trajectory.generated[1]

    point_1.x = msg.point_1.position.x
    point_1.y = msg.point_1.position.y
    point_1.z = msg.point_1.position.z
    point_1.yaw = msg.point_1.yaw
    point_1.type_mask = msg.point_1.type_mask


    point_2.x = msg.point_2.position.x
    point_2.y = msg.point_2.position.y
    point_2.z = msg.point_2.position.z
    point_2.yaw = msg.point_2.yaw
    point_2.type_mask = msg.point_2.type_mask
    point_2.point_valid = msg.point_valid

    #print('-----')
    #print(msg.point_valid[0])
    #print('-----')

    #generated = []
    #generated.append(msg.point_1.position.x)
    #generated.append(msg.point_1.position.y)
    #generated.append(msg.point_1.position.z)
    #generated.append(msg.point_1.yaw)
#
    #trajectory.generated = generated
    #print("/trajectory/generated Point 1: ")
    #print(msg.point_1.position)
    #print("####")

def subscribe_to_topics():
    rospy.Subscriber("/mavros/trajectory/desired", mavros_msgs.msg.Trajectory, show_msg)
    rospy.Subscriber("/mavros/trajectory/generated", mavros_msgs.msg.Trajectory, show_generated )
    

if __name__ == "__main__":
    rospy.init_node('Show_Trajectory')

    point_1_g = Points()
    point_2_g = Points()

    point_1_d = Points()

    trajectory = Trajectory( [point_1_d] , [point_1_g, point_2_g]  , 0)
    subscribe_to_topics()

    while not rospy.is_shutdown():
        try:
            trajectory.show_msg()
            rospy.sleep(0.08)

        except KeyboardInterrupt():
            sys.exit()
        #rospy.spin()