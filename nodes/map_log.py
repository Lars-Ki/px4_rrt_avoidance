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
import nav_msgs
from nav_msgs.msg import OccupancyGrid 
import numpy as np
import mavros_msgs
import time
import xlsxwriter

class Map:
    def __init__(self,x,y,resolution, grid):
        self.x = x
        self.y = y
        self.resolution = resolution        #[width, height, pixel_size]
        self.grid = grid


def save_as_grid(data):

    map.grid = np.zeros((map.resolution[1], map.resolution[0]))  # Zeile x Spalte #ACHTUNG!!!!!!!!!!!!!!!!!!!!!!!!
    j = 0 #height of map 
    i = 0  #width of map 
    k = 0 # position in Map_matrix
    while (j <= map.resolution[1]-1):
        while (i <= map.resolution[0] -1 ): 
            if (k <= len(data) - 1):

                map.grid[j][i] = data[k]
                k = k +1
            i = i +1 
            
        if (i == map.resolution[0]):
            i = 0 
        j = j +1 
    
    print(map.grid)
    

def split_map(map_1):
    print(map_1.info.origin.position)
    map.x = map_1.info.origin.position.x
    map.y = map_1.info.origin.position.y
    info = map_1.info
    map.resolution = [info.width, info.height, info.resolution]
    data = map_1.data
    save_as_grid(data)
    

def sub_to_tobic():
    print("in sub")
    msg = rospy.wait_for_message("/local_map", nav_msgs.msg.OccupancyGrid)
    split_map(msg)


def save_exel():
    workbook = xlsxwriter.Workbook('Grid_map.xlsx')
    worksheet = workbook.add_worksheet()
    header_list = ["width", "height", "pixel_size", "x", "y"]

    row = 0
    col = 0
    while row <= len(header_list)-1:
        worksheet.write(row, col, header_list[row])
        
        if row> len(map.resolution)-1:
            if header_list[row] == "x":
                worksheet.write(row, col+1, map.x) 
            if header_list[row] =="y":
                worksheet.write(row, col+1, map.y)       
        else:
            worksheet.write(row, col+1, map.resolution[row])

        row = row +1
    worksheet.write(row, col, "Grid_map")
    row = row +1
    #save grid 
    i = map.resolution[1]-1 #height
    while i >= 0:
        j = 0 #width
        while j <= map.resolution[0]-1:
            worksheet.write(row +i, j, map.grid[len(map.grid) -1 -i][j])
            j = j +1
        i = i-1
    workbook.close()
    print("done")

    

if __name__ == "__main__":
    rospy.init_node("log_local_map")
    map = Map(-2, -2,resolution=[4,4,0.5],grid=[[0,0,0,0], [0,0,0,0], [0,0,0,0], [0,0,0,0]])
    sub_to_tobic()
    rospy.sleep(1)

    while not rospy.is_shutdown():
        try:
            save_exel()
            sys.exit()

        except KeyboardInterrupt():
            sys.exit()
        
    rospy.spin()