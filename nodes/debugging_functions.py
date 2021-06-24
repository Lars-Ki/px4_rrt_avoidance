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

import mavros_msgs
from nav_msgs.msg import Odometry, Path
from px4_rrt_avoidance.msg import Path_coll
from geometry_msgs.msg import PoseStamped


def show_checked_path(map_origin, map_matrix, resolution, checked_list):
    #map_origin = [x, y]
    #map_matrix = [ k_1=[n_1, ...., n_width] ....k_height=[n_1, ...., n_height]]
    #resolution = {"width": ..., "height": ...., "pixel_size": .....}
    #checked_list = [[voxel_ind_1][voxel_ind_2]..... ]

    map_matrix = map_matrix
    map_origin_x = map_origin[0]
    map_origin_y = map_origin[1]
    pixel_size = resolution.get('pixel_size')
    width = resolution.get('width')
    height = resolution.get('height')
    
    for voxel in checked_list:
        if voxel[0] < len(map_matrix[0]) and voxel[1] <len(map_matrix):
            #
            if map_matrix[voxel[1]][voxel[0]] == -1:
                map_matrix[voxel[1]][voxel[0]] = 100
                continue

            if map_matrix[voxel[1]][voxel[0]] == 0:
                #print("in zero")
                map_matrix[voxel[1]][voxel[0]] = -1 #99 instead of 100: vehicle stops 
                continue

            #if map_matrix[voxel[1]][voxel[0]] == 100:
            #    map_matrix[voxel[1]][voxel[0]] = 0
            #    continue
            
            else: #when ==80
                map_matrix[voxel[1]][voxel[0]] = 100
                continue

        
    #make data for publishing map
    i= 0
    data = []
    while i <= len(map_matrix) -1:
        j = 0 #width
        while j <= len(map_matrix[0]) -1:
            data.append(map_matrix[i][j])
            #if map_matrix[i][j] == 100:
            #    print("width: " +str(j) + " height: " +str(i))

            j = j +1
        i = i +1

    grid_msg = OccupancyGrid()
    grid_msg.header.stamp = rospy.get_rostime()
    grid_msg.header.frame_id = "map"

    grid_msg.info.width = width
    grid_msg.info.height = height
    grid_msg.info.resolution = pixel_size

    grid_msg.info.origin.position.x = map_origin_x
    grid_msg.info.origin.position.y = map_origin_y
    grid_msg.info.origin.orientation.x = 0
    grid_msg.info.origin.orientation.y = 0
    grid_msg.info.origin.orientation.z = 0
    grid_msg.info.origin.orientation.w = 0


    grid_msg.data = data

    if len(data) >= 1:
        pub = rospy.Publisher('path_map', OccupancyGrid, queue_size=10)
        pub.publish(grid_msg)
        #print(len(data))
        #print("widthxheight: " + str(width*height))


    ############################################################
def pub_node_list(node_list, origin, height, width, pixel_size):
    
    points = []
    for element in node_list:
        #position = [element.x, element.y]
        x_ind, y_ind = get_voxel_inds(element.x, element.y, width, height, origin , pixel_size)
        voxel = [x_ind, y_ind]
        #if x_ind < local_map.resolution.get('width')
        points.append(voxel)

    #pub = rospy.Publisher('node_map', nav_msgs.msg.OccupancyGrid, queue_size=10 )
    print(points)

    matrix = np.zeros((height, width))
    for voxel in points:
        matrix[voxel[1]-1][voxel[0]-1] = 100

    i= 0
    data = []
    while i <= len(matrix) -1:
        j = 0 #width
        while j <= len(matrix[0]) -1:
            data.append(matrix[i][j])

            j = j +1
        i = i +1
    grid_msg = OccupancyGrid()
    grid_msg.header.stamp = rospy.get_rostime()
    grid_msg.header.frame_id = "map"

    grid_msg.info.width = len(matrix[0])
    grid_msg.info.height = len(matrix)

    grid_msg.info.resolution = pixel_size

    grid_msg.info.origin.position.x = origin[0]
    grid_msg.info.origin.position.y = origin[1]
    grid_msg.info.origin.orientation.x = 0
    grid_msg.info.origin.orientation.y = 0
    grid_msg.info.origin.orientation.z = 0
    grid_msg.info.origin.orientation.w = 0

    grid_msg.data = data

    pub = rospy.Publisher('node_map', nav_msgs.msg.OccupancyGrid, queue_size=10 )
    pub.publish(grid_msg)

def get_voxel_inds(x , y,  width, height, origin, pixel_size):

    x_origin = origin[0]
    y_origin = origin[1]
    resolution = pixel_size
    if (x >= x_origin and y >= y_origin):
        x_int = 0
        while (x_int +1 <= width ):
            if (x_origin + resolution + (x_int * resolution) >= x):
                break
            x_int = x_int +1
        y_int = 0
        while (y_int <= height ):
            if (y_origin + resolution + (y_int*resolution) >= y) :
                break
            y_int = y_int +1

    else:
        print("Not in map")
        x_int = float('NaN')   #CAN CAUSE ERROR???
        y_int = float('NaN')   #CAN Cause ERROR???? 
    return x_int , y_int

    ############################################################

def visualize_path(path, topic_name, start_pos):   #msg = Topic: /safe_path 
    path_list = []
    path_list.append([start_pos[0], start_pos[1]])
    for wp in path:
        path_list.append(wp)


    msg = Path()
    msg.header.frame_id = "/map"
    msg.header.stamp = rospy.Time.now()
    #print(path_list)
    if len(path_list) >= 1:
        for position in path_list:
            pose = PoseStamped()
            pose.pose.position.x = position[0]
            pose.pose.position.y = position[1]
            pose.pose.position.z = 0
            msg.poses.append(pose)


        pub = rospy.Publisher('/' + topic_name, Path, queue_size=1)
        pub.publish(msg)