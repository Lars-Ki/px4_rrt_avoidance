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
from mavros_msgs.msg import PositionTarget
from mavros_msgs.msg import Trajectory
from px4_rrt_avoidance.msg import Path
import numpy as np
import mavros_msgs
import time
import xlsxwriter
import atexit

# local_position/pose: 30hz
# 

class Measure_Data:

    def __init__(self, started, goal_reached, start_time, end_time, goal, flight_path, msg_counter, y_dis_list=[], d_time_list=[], global_x_path = [], Weg_Integral = 0, delta_dis_list =[]):
        self.started = started              #False/True
        self.goal_reached = goal_reached    #False/True
        self.start_time = start_time
        self.end_time = end_time
        self.goal = goal                    # [x_goal, y_goal]
        self.flight_path = flight_path      # [[x_1, y_1, z_1, time], [x_2,..], ...]
        self.msg_counter = msg_counter
        self.y_dis_list = y_dis_list
        self.d_time_list = d_time_list
        self.global_x_path = global_x_path
        self.Weg_Integral = Weg_Integral
        self.delta_dis_list = delta_dis_list

    def log_weg_Integral_data(self, path_x, path_y, ges_Integral):
        self.global_x_path = path_x
        self.delta_dis_list = path_y
        self.Weg_Integral = ges_Integral

def measure_time(msg):
    point = msg.point_2.position
    if data.started == False and point.x >= 49:
        data.start_time = msg.header.stamp.secs +( msg.header.stamp.nsecs*(10**(-9)))
        data.started = True 
        data.goal = [point.x, point.y]
        print("goal: " + str(data.goal[0]) + "  " + str(data.goal[1]))
        print("Started measuring!")
        print ("current time: "+ str(data.start_time))

def log_position(position):
    pos = position.pose.position
    distance_to_goal = calc_distance([pos.x, pos.y], data.goal)

    if (distance_to_goal <= 0.5 and data.goal_reached == False):
        data.goal_reached = True
        data.end_time = position.header.stamp.secs + (position.header.stamp.nsecs*(10**(-9)) )
        print("reached goal!")
        sim_time = data.end_time - data.start_time
        print("Simulation time: ")
        print(str(sim_time))
       # print(data.flight_path)
    
    if (data.started == True and data.goal_reached == False):
        time = position.header.stamp.secs + (position.header.stamp.nsecs*(10**(-9)) )
        next_step = [pos.x, pos.y, pos.z, time]
        data.flight_path.append(next_step)



def calc_distance(start, end):
    x_1, y_1 = start[0], start[1]
    x_2, y_2 = end[0], end[1]

    d_x = x_2 - x_1
    d_y = y_2 - y_1
    dist = math.sqrt(d_x**2 + d_y**2)
    #rospy.sleep(0.1)
    return dist

def counter(msg):
    #print(msg)
    if data.started == True and data.goal_reached == False:
        data.msg_counter = data.msg_counter +1


def sub_to_topic():
    rospy.Subscriber("/mavros/trajectory/desired", Trajectory, measure_time)
    rospy.Subscriber("/mavros/local_position/pose", geometry_msgs.msg.PoseStamped, log_position)
    rospy.Subscriber("/save_path", Path, counter )

def calculate_itegrals():
    print("finished... starting calculating")
    data.flight_path[0][3] = (data.flight_path[1][3] - (1/30))
    #data.flight_path[0].append(data.flight_path[1][3] - (1/30) ) #runs at 30 hz, idealy drone was 1/30 sec before the start at the start node 

    #Start always at [0,0]
    alpha = math.tan(data.goal[1]/data.goal[0]) #rad
    global_path = [[0,0]]   #start = [0,0]
    path_resolution = 0.1
    last_element = [global_path[0][0], global_path[0][1]]
    remaining_distance_to_goal = calc_distance(last_element, data.goal)
    while remaining_distance_to_goal >= 0.15: #while last element in global_path <= goal's x position: add x,y value to global_path
        
        x = last_element[0] + math.cos(alpha)*path_resolution
        y = last_element[1] + math.sin(alpha)*path_resolution

        global_path.append([x,y])
        #print(global_path) 

        last_element = [global_path[len(global_path)-1][0], global_path[len(global_path)-1][1]]
        remaining_distance_to_goal = calc_distance(last_element, data.goal)

    #now add the goal_node
    global_path.append([data.goal[0], data.goal[1]])

    #calculate the closest point to each global_path point if drone passed current x point
    #otherwise define min distance to min_distance_value
    max_distance_value = 25
    List_of_min_dist = [max_distance_value]*(len(global_path))
    i = 0
    while i <= len(global_path)-1:
        close_points = []
        current_x_value = global_path[i][0]
        z = 0
        while z <= len(data.flight_path)- 1:
            if (data.flight_path[z][0] >= current_x_value - path_resolution and data.flight_path[z][0] <= current_x_value + path_resolution):
                close_points.append([data.flight_path[z][0],data.flight_path[z][1]])
            z = z +1

        local_dis_list = []
        for point in close_points:
            local_dist = calc_distance(point, global_path[i])
            local_dis_list.append(local_dist)


        if len(local_dis_list) > 0:
            if List_of_min_dist[i] >= min(local_dis_list):
                List_of_min_dist[i] = min(local_dis_list)

        else:
            print("length of local list = 0")
            print("current x: " + str(current_x_value))
            if len(close_points) >0:
                print("length close_points: " + str(len(close_points)))
                print("distance example: " + str(calc_distance(close_points[0], [ global_path[i]])) )

        i = i+1

    if data.goal_reached == True: #last points might still be 25 due to acceptance radius
        
        k = len(List_of_min_dist)  -11
        print(k - len(List_of_min_dist))
        while k <= len(List_of_min_dist) -1:
            #print(k)
            if List_of_min_dist[k] == 25:
                List_of_min_dist[k] = List_of_min_dist[k-1] + path_resolution
            k = k+1
    #print(List_of_min_dist)
    Weg_integral = 0
    i = 0
    while i <= len(List_of_min_dist)-1:
        Weg_integral = Weg_integral + (List_of_min_dist[i]*path_resolution)
        i = i+1
    
    print("ges Weg Integral: " +str(Weg_integral) )
    path_x = []
    path_y = []
    for x_value in global_path:
        path_x.append(x_value[0])
    for y_value in List_of_min_dist:
        path_y.append(y_value)
    
    if len(path_y) == len(path_x):
        print("Length are fine!")
    else:
        print("Not identical!!!")
    
    data.log_weg_Integral_data(path_x, path_y, Weg_integral)

    #Integral = Sum(x(t_i) * d_t)
    Zeit_Integral = 0
    i = 0
    d_time_list = []
    y_distance_list = []

    while (i <= len(data.flight_path) -1):
        drone_x = data.flight_path[i][0]
        y_dis = float('nan')
        d_time = float('nan')
        if (drone_x < 0 or drone_x > data.goal[0]):
            #print("exceeds normal path")
            if drone_x < 0:
                y_dis = calc_distance([data.flight_path[i][0],data.flight_path[i][1]], [0,0])
            if drone_x > data.goal[0]:
                y_dis = calc_distance( [data.flight_path[i][0],data.flight_path[i][1] ], data.goal)
        else:
            # d_y: original path distance to y = 0
            d_y = math.tan(alpha)*drone_x
            y_dis = data.flight_path[i][1] - d_y
        #print("--")
        if i == len(data.flight_path) -1:
            d_time = 1/30   #publisched at 30hz
        else:
            d_time = data.flight_path[i+ 1][3] - data.flight_path[i][3]

        
        d_time_list.append(d_time)
        y_distance_list.append(abs(y_dis))

        y_dis = abs(y_dis)  #only positive 
        Zeit_Integral = Zeit_Integral + (y_dis*d_time)
        i = i+1
    data.y_dis_list = y_distance_list
    data.d_time_list = d_time_list
    print("Zeit_Integral calculated")
    return Zeit_Integral


def calc_travel_distance():
    print("start travel_dis")
    traveled_distance = 0
    i = 0 
    while (i <= len(data.flight_path) -2):
        path = data.flight_path
        start = [path[i][0], path[i][1]]
        end = [path[i+1][0], path[i+1][1]]

        distance = calc_distance(start, end)        
        traveled_distance = traveled_distance + distance

        i = i+1
    print("traveled distance calculated")
    return traveled_distance

def write_data(Zeit_Integral, sim_time, traveled_distance, mean_speed, ave_path_time, msg_counter, distance_orig):
    workbook = xlsxwriter.Workbook('Test1.xlsx')
    worksheet = workbook.add_worksheet()
    header_results = [Zeit_Integral, sim_time, traveled_distance, mean_speed, ave_path_time, msg_counter, distance_orig]
    header_list =    ['Zeit_Integral [ms]', 'Sim.-Time [s]', 'Traveled distance [m]','Mean Speed [m/s]', 'Average Path-Time [s]', 'Messages Counted', 'Length of original Path [m]']
    col_list_name = ['Zeit', 'x', 'y', 'z', 'd_time', 'y_dis',]

    row = 0
    col = 0
    start_measure_time = data.flight_path[0][3]

    if data.goal_reached == False:
        z = 0
        while z <= 6:
            worksheet.write(z, 3, "Mission was aborted")
            z = z +1
    

    while (row <= len(header_list) -1):
        worksheet.write(row, col, header_list[row])
        worksheet.write(row, col +1, header_results[row] )

        row = row +1
    row = row +1  # a row extra space
    
    while col <= len(col_list_name) - 1:

        worksheet.write(row, col, col_list_name[col]) 
        col = col +1 

    row = row +1
    col = 0
    i = 0
    while i <= len(data.flight_path) -1:
        #print("i: " + str(i))
        time = data.flight_path[i][3] - start_measure_time
        worksheet.write(row, 0, time)
        worksheet.write(row, 1, data.flight_path[i][0])
        worksheet.write(row, 2, data.flight_path[i][1])
        worksheet.write(row, 3, data.flight_path[i][2])

        if i >= len(data.y_dis_list) -1:
            break
        else:
            worksheet.write(row, 4, data.d_time_list[i])
            worksheet.write(row, 5, data.y_dis_list[i])


        i = i +1
        row = row +1
###################################################################################
    worksheet.write(6, 7, 'Weg Integral [m*m]')
    worksheet.write(6, 8, data.Weg_Integral)
    worksheet.write(8, 7, 'global x_path')
    worksheet.write(8, 8, 'y_dis to global_path')
    i = 9 
    while (i-9 <= len(data.global_x_path) -1):
        worksheet.write(i, 7, data.global_x_path[i-9])
        worksheet.write(i, 8, data.delta_dis_list[i-9])
        i = i+1
    
    workbook.close()

def Interrupt_function():
    print("abort mission: log data...")
    Zeit_Integral = calculate_itegrals()
    traveled_distance = calc_travel_distance()
    distance_orig = calc_distance([0,0], data.goal)

    #last_msg = rospy.wait_for_message("/mavros/local_position/pose", geometry_msgs.msg.PoseStamped, timeout=None)
    #data.end_time = last_msg.header.stamp.secs + (position.header.stamp.nsecs*(10**(-9)) )
    data.end_time = data.flight_path[len(data.flight_path) -1][3] + (1/30)
    sim_time = data.end_time - data.start_time

    mean_speed = traveled_distance/sim_time
    ave_path_time = 0
    print("everything exept if statements")
    if data.msg_counter == 0:
        print("average time to calc new path: " +str(0))
    else:
        ave_path_time = sim_time/data.msg_counter
        print("average time to calc new path: " +str(sim_time/data.msg_counter))
    print("writing data")
    write_data(Zeit_Integral, sim_time, traveled_distance, mean_speed, ave_path_time, data.msg_counter, distance_orig)
    sys.exit()




if __name__ == "__main__":
    rospy.init_node("measure_data")
    data = Measure_Data(False, False, time.time(), time.time(), [50,0], [[0,0,15,0]], 0)
    sub_to_topic()

    while not rospy.is_shutdown():
        try:
            #print(data.started)
            #print(data.goal_reached)
            if (data.started == True and data.goal_reached == True):
                Zeit_Integral = calculate_itegrals()
                traveled_distance = calc_travel_distance()
                print("msg counted: " + str(data.msg_counter))
                print("Zeit_Integral")
                print(Zeit_Integral)
                print("traveled ditsance: " + str(traveled_distance))
                distance_orig = calc_distance([0,0], data.goal)
                print("orig. Path_length" + str(distance_orig ))
                sim_time = data.end_time - data.start_time
                print("Sim. time: " +str(sim_time))
                mean_speed = traveled_distance/sim_time
                print("mean speed [m/s]: " +str(mean_speed))
                ave_path_time = 0
                if data.msg_counter == 0:
                    print("average time to calc new path: " +str(0))
                else:
                    ave_path_time = sim_time/data.msg_counter
                    print("average time to calc new path: " +str(sim_time/data.msg_counter))

                write_data(Zeit_Integral, sim_time, traveled_distance, mean_speed, ave_path_time, data.msg_counter, distance_orig)
                sys.exit()
        except KeyboardInterrupt():
            print("Test1")
            #Interrupt_function()
            #rospy.sleep(1)
            sys.exit()
        
        #atexit.register(Interrupt_function)
    rospy.spin()
    atexit.register(Interrupt_function)