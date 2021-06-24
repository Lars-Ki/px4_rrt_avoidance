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
from px4_rrt_avoidance.msg import Path_coll
from px4_rrt_avoidance.msg import Maximizer
from px4_rrt_avoidance.msg import RRT_Star_Call
import mavros_msgs
import time
from debugging_functions import show_checked_path
from debugging_functions import pub_node_list


class Drone():
    def __init__(self, drone_pose, desired_altitude):
        self.drone_pose = drone_pose
        self.desired_altitude = desired_altitude

    def set_drone_position(self, msg):
        position = msg.pose.position
        orientation =  msg.pose.orientation
        x = position.x 
        y = position.y
        z = position.z
        pitch = orientation.x
        yaw = orientation.y
        roll = orientation.z
        self.drone_pose = [x, y, z, roll, pitch, yaw]
        rrt_star.start = rrt_star.Node(x , y)

        x_ind , y_ind = get_voxel_inds(x, y)
        try:
            local_map_value = local_map.map_matrix[y_ind][x_ind] 
            if local_map_value >= 30:
                new_x, new_y = local_map.search_closest_save_point([x_ind, y_ind])
                rrt_star.start = rrt_star.Node(new_x, new_y)
        except IndexError:
            rrt_star.start = rrt_star.Node(x , y)


class Local_Map():
    def __init__(self, map_origin, resolution, map_matrix, target_position, maximizer, start_planning = False ):
        self.resolution = resolution            # width, height, pixel_size
        self.map_origin = map_origin            # [x,y,z, x,y,z,w] x, y, z and orientation: x, y, z, w
        self.map_matrix = map_matrix            # [[-1, -1, 0, 100], [-1, -1, -1, 0], [....]...] -1: unknown; 0: not occupied; 100: occupied
        self.target_position = target_position          # [x, y, z]
        self.maximizer = maximizer
        self.start_planning = start_planning    # starts planning when requested by Masternode 

    def local_map_splitter(self, map):

        info = map.info
        width = info.width
        height = info.height
        position = info.origin.position
        origin = [position.x, position.y] 
        pixel_size = round(map.info.resolution,3)
        map_vector = map.data

        map_matrix = np.zeros((height, width))

        i = 0 #height
        k = 0 # position in Map_matrix 
        while (i <= height -1):
            j = 0 
            while (j <= width -1):
                map_matrix[i][j] = map_vector[k] 
                k = k +1
                j = j +1
            i = i +1
        self.map_matrix = map_matrix
        self.resolution = {"width": width, "height": height, "pixel_size": pixel_size}
        self.map_origin = origin

        rrt_star.min_rand_height = (self.map_origin[1] / local_map.resolution.get('pixel_size')) +1                         # reduce search space to nod fly at edge
        rrt_star.max_rand_height = ((self.map_origin[1] + height*pixel_size) / local_map.resolution.get('pixel_size')) -1
        rrt_star.min_rand_width = ((self.map_origin[0]) / local_map.resolution.get('pixel_size')) +1
        rrt_star.max_rand_width = ((self.map_origin[0] + width*pixel_size)  / local_map.resolution.get('pixel_size') ) -1

    def set_target(self, target_position):

        rrt_star.start_planning = target_position.start_planning
        if target_position.start_planning == False:
            local_map.maximizer = 0
            pub_map_maximizer(local_map.maximizer)
        
        x_target = target_position.target_position[0]
        y_target = target_position.target_position[1]
        z_target = target_position.target_position[2]
        
        print("target_pos: x: " + str(x_target) + " y: " + str(y_target))
        
        x_ind, y_ind = get_voxel_inds(x_target, y_target)
        target_voxel = [x_ind, y_ind] #x_ind and y_nd at begining = nan
        #CAUTION
        print("Index: " + str(target_voxel))
        try:
            local_map_value = local_map.map_matrix[target_voxel[1]][target_voxel[0]] 
        except IndexError:
            print("Index was out of bounds!!!")
            local_map_value = 0     #we assume that unknwon space is free

        if ((local_map_value >= 30) and drone_1.drone_pose[2] > 1):
            #print("Waypoint too close to obstacle")
            print("waypoint inside occupied cell")
            x_target, y_target = self.search_closest_save_point(target_voxel)

        if math.isnan(z_target) == True:
            z_target = 0 
        drone_1.desired_altitude = z_target

        if( abs(x_target -drone_1.drone_pose[0]) >= 400  or abs(y_target - drone_1.drone_pose[1]) >= 400):
            #print("Next waypoint is too far away")
            x_target = round(drone_1.drone_pose[0],1)
            y_target = round(drone_1.drone_pose[1], 1)


        local_map.target_position = [x_target, y_target, z_target]
        rrt_star.end = rrt_star.Node(x_target , y_target)
        rrt_star.goal_node.x = x_target
        rrt_star.goal_node.y = y_target
       # test_print()
       

    def search_closest_save_point(self, t_voxel):
        search_radius = 7 #PARAMETER number of voxels to check in radius/square
                          #should not be higher than minimum of local_map radius
        width_start = t_voxel[0] - search_radius
        height_start = t_voxel[1] - search_radius
        available_voxels = []

        i = height_start
        while (i <= height_start + (2*search_radius) -1 and i <= local_map.resolution.get('height')-1 ):
            j = width_start
            while (j <= width_start + (2*search_radius) -1 and j <= local_map.resolution.get('width')-1 ):

                if local_map.map_matrix[i][j] == 0 or local_map.map_matrix[i][j] == -1 : #there must be free space when an obstacle was detected
                    available_voxels.append([j,i]) #[width, height] 
                j = j +1
            j = width_start
            i = i + 1

        #search for closest voxel:
        distance_list = []
        z = len(available_voxels) - 1
        while z >=0:
            voxel = available_voxels[z]
            d_x = distance(t_voxel[0], voxel[0])
            d_y = distance(t_voxel[1], voxel[1])
            true_distance = math.sqrt(d_x**2 + d_y**2)
            distance_list.append(true_distance)
            z = z-1

        min_dis = min(distance_list)
        min_index = distance_list.index(min_dis)

        save_voxel = available_voxels[min_index]


        save_geo = self.get_geo_pos(save_voxel)
        return save_geo[0], save_geo[1]

    @staticmethod
    def get_geo_pos(voxel):
        x_geo = local_map.map_origin[0] + voxel[0]*local_map.resolution.get('pixel_size')
        y_geo = local_map.map_origin[1] + voxel[1]*local_map.resolution.get('pixel_size')

        save_geo = [x_geo, y_geo]
        return save_geo




############################ RRT STAR ORIGINE NOTICE #################################
#   Copyright (c) 2016 - 2021 Atsushi Sakai

#   Permission is hereby granted, free of charge, to any person obtaining a copy
#   of this software and associated documentation files (the "Software"), to deal
#   in the Software without restriction, including without limitation the rights
#   to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
#   copies of the Software, and to permit persons to whom the Software is
#   furnished to do so, subject to the following conditions:

#   The above copyright notice and this permission notice shall be included in all
#   copies or substantial portions of the Software.

#   THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
#   IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
#   FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
#   AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
#   LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
#   OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
#   SOFTWARE.
#######################################################################################
# For more details vist: https://github.com/AtsushiSakai/PythonRobotics               #
#######################################################################################

class RRT_Star(object):

    class Node():
        def __init__(self, x, y):
            super(RRT_Star).__init__(RRT_Star)  #ERROR WEIL PYTHON2.7 not Python3 -> switch to ROS2 (not recommended for now)
            self.cost = 0.0
            self.x = x
            self.y = y
            self.path_x = []
            self.path_y = []
            self.parent = None 

    def __init__(self, start, goal, rand_area_width,rand_area_height,  
                expand_dis = 6,
                path_resolution = 1.5,
                goal_sample_rate = 20,          #[%] will check goal node
                max_iter = 500,                 #CHANGE VALUE IN FUTURE
                connect_circle_dist = 7.0):

        super(RRT_Star, self).__init__( )

        self.connect_circle_dist = connect_circle_dist
        self.goal_node = self.Node(goal[0], goal[1])

        #from RRT planner:
        self.start = self.Node(start[0], start[1])
        self.end = self.Node(goal[0], goal[1])      
        self.min_rand_width = rand_area_width[0] / local_map.resolution.get('pixel_size')       #later multiplied by pixel size 
        self.max_rand_width = rand_area_width[1] / local_map.resolution.get('pixel_size')       #later multiplied by pixel size 
        self.min_rand_height = rand_area_height[0] / local_map.resolution.get('pixel_size')     #later multiplied by pixel size 
        self.max_rand_height = rand_area_height[1] / local_map.resolution.get('pixel_size')     #later multiplied by pixel size 
        self.expand_dis = expand_dis
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate        #percent: checks if goal can be reached
        self.max_iter = max_iter
        self.node_list = []
        self.start_planning = False    # starts planning when requested by Masternode 

    def calc_dist_to_goal(self, x, y):
        dx = x - self.end.x
        dy = y - self.end.y
        return math.hypot(dx, dy)


    def planning(self, search_until_max_iter = False):

        self.node_list = [self.start]
        for i in range(self.max_iter):

            rnd = self.get_random_node()
            nearest_ind = self.get_nearest_node_index(self.node_list, rnd)            
            new_node = self.steer(self.node_list[nearest_ind], rnd, self.expand_dis)
            near_node = self.node_list[nearest_ind]
            new_node.cost = near_node.cost + \
                math.hypot(new_node.x-near_node.x, new_node.y-near_node.y)
            
            if self.check_collision(new_node):             
                near_inds = self.find_near_nodes(new_node)                                 

                node_with_updated_parent = self.choose_parent(new_node, near_inds)
                if node_with_updated_parent:
                    self.rewire(node_with_updated_parent, near_inds)
                    self.node_list.append(node_with_updated_parent)
                else:
                    self.node_list.append(new_node)


                if( not search_until_max_iter and new_node):

                    last_index = self.search_best_goal_node() 
                    if last_index is not None:

                        return self.generate_final_course(last_index)   #line 885 in RRT Star



        last_index = self.search_best_goal_node()

        if last_index is not None:

            return self.generate_final_course(last_index)

        return None


    def generate_final_course(self, goal_ind):

        path = [[self.end.x, self.end.y]]
        node = self.node_list[goal_ind]
        while node.parent is not None:
            path.append([node.x, node.y])
            node = node.parent

        path.append([node.x, node.y])

        return path


    def search_best_goal_node(self):
        dist_to_goal_list = [self.calc_dist_to_goal(n.x, n.y) for n in self.node_list]
        goal_inds = [dist_to_goal_list.index(i) for i in dist_to_goal_list if i <= self.expand_dis]

        safe_goal_inds=[]

        for goal_ind in goal_inds:

            t_node = self.steer(self.node_list[goal_ind], self.goal_node)


            if self.check_collision(t_node):                  
                safe_goal_inds.append(goal_ind)

            if not safe_goal_inds:
                return None 

            min_cost = min([self.node_list[i].cost for i in safe_goal_inds])
            for i in safe_goal_inds:
                if self.node_list[i].cost == min_cost:
                    return i

            return None  

    def get_random_node(self):
        if random.randint(0, 100) > self.goal_sample_rate:
            rnd_1 = random.randint(self.min_rand_width, self.max_rand_width) * round(local_map.resolution.get('pixel_size'),2)
            rnd_2 = random.randint(self.min_rand_height, self.max_rand_height) * round(local_map.resolution.get('pixel_size'),2)
            rnd = self.Node(rnd_1, rnd_2)

        else: #goal point sampling
            rnd = self.Node(self.end.x, self.end.y)

        return rnd

    def rewire(self, new_node, near_inds):
        for i  in near_inds:
            near_node = self.node_list[i]
            edge_node = self.steer(new_node, near_node)
            if not edge_node:
                continue

            edge_node.cost = self.calc_new_cost(new_node, near_node)

            no_collision = self.check_collision(edge_node)  
            improve_cost = near_node.cost > edge_node.cost

            if no_collision and improve_cost:
                near_node.x = edge_node.x
                near_node.y = edge_node.y
                near_node.cost = edge_node.cost
                near_node.path_x = edge_node.path_x
                near_node.path_y = edge_node.path_y
                near_node.parent = edge_node.parent
                self.propagate_cost_to_leaves(new_node)    


    def propagate_cost_to_leaves(self, parent_node):
        for node in self.node_list:
            if node.parent == parent_node:
                node.cost = self.calc_new_cost(parent_node, node)
                self.propagate_cost_to_leaves(node)


    def find_near_nodes(self, new_node):
        nnode = len(self.node_list) + 1
        r = self.connect_circle_dist * math.sqrt((math.log(nnode) / nnode ))

        if hasattr(self, 'expand_dis'):        #hasattr(): true if object has the beforementioned attribute
            r = min(r, self.expand_dis)

        dist_list = [(node.x -new_node.x)**2 + (node.y - new_node.y)**2 for node in self.node_list]     
        near_inds = [dist_list.index(i) for i in dist_list if i <= r**2 ]     #r^2 = ~866

        return near_inds



    def choose_parent( self, new_node, near_inds):
        if not near_inds:
            return None 

        costs = []
        for i in near_inds:
            near_node = self.node_list[i]
            t_node = self.steer(near_node, new_node)
            if t_node and self.check_collision(t_node):
                costs.append(self.calc_new_cost(near_node, new_node))
 
                           
            else:
                costs.append(float("inf"))       #the cost of collision node

        min_cost = min(costs)

        if min_cost == float("inf"):
            return None 
        
        min_ind = near_inds[costs.index(min_cost)]
        new_node = self.steer(self.node_list[min_ind], new_node)
        new_node.cost = min_cost 

        return new_node



    def calc_new_cost(self, from_node, to_node):
        d, _ = self.calc_distance_and_angle(from_node, to_node)
        return from_node.cost + d 




    def steer(self, from_node, to_node, extend_length=float('inf')):

        new_node = self.Node(from_node.x, from_node.y)
        d, theta = self.calc_distance_and_angle(new_node, to_node)              

        new_node.path_x = [new_node.x]
        new_node.path_y = [new_node.y]

        if extend_length > d:
            extend_length = d

        n_expand = int(math.floor(extend_length     / self.path_resolution)) 
        

        for _ in range(n_expand):
            new_node.x += self.path_resolution * math.cos(theta)
            new_node.y += self.path_resolution * math.sin(theta)

            new_node.path_x.append(new_node.x)
            new_node.path_y.append(new_node.y)

        d, _ = self.calc_distance_and_angle(new_node, to_node)

        if d <= self.path_resolution:
            new_node.path_x.append(to_node.x)
            new_node.path_y.append(to_node.y)
            new_node.x = to_node.x
            new_node.y = to_node.y
            

        new_node.parent = from_node

        return new_node



    @staticmethod
    def calc_distance_and_angle(from_node, to_node):  

        #print(from_node)
        dx = (to_node.x) - (from_node.x)
        dy = to_node.y - from_node.y

        d = math.hypot(dx, dy)
        theta = math.atan2(dy, dx)
        return d, theta 

    @staticmethod
    def get_nearest_node_index(node_list, rnd_node):
        dlist = [(node.x -rnd_node.x)**2 + (node.y - rnd_node.y)**2 for node in node_list]
        minind = dlist.index(min(dlist))

        return minind
 
    
    @staticmethod
    def remove_node_element(node, i):
        del node.path_x[i]
        del node.path_y[i]


    #@staticmethod
    def check_collision(self, node):
        if node is None:
            return False # collision


        i = 0

        while (i <= len(node.path_x) -1):

            x_node = node.path_x[i]
            y_node = node.path_y[i]
            in_map = check_node(x_node, y_node)

            if in_map == False:

                self.remove_node_element(node, i)
                break
            x_int, y_int = get_voxel_inds(x_node, y_node)
            if (math.isnan(x_int) == True):
                i = i+1
                continue
            if ( x_int >= local_map.resolution.get('width') or y_int >= local_map.resolution.get('height')):
                break
            
            try:
                if (local_map.map_matrix[y_int][x_int] >= 30 ):
                    return False 
            except IndexError:
                pass
            i = i +1

        return True 

    @staticmethod
    def check_2node_collision( start_node, end_node ):
        x_1 , y_1 = start_node.x, start_node.y
        x_2 , y_2 = end_node.x, end_node.y

        simple_angle, angle = calc_angle([x_1, y_1], [x_2, y_2])

        d_x = distance(x_1, x_2)
        d_y = distance(y_1, y_2)

        if(d_x is None or d_y is None): #Needed for Take-off
            return True
        if (abs(d_x) <= local_map.resolution.get('pixel_size') and abs(d_y) <= local_map.resolution.get('pixel_size')):

            return True

        start_position = [x_1, y_1]
        next_node = [x_2, y_2]
        list_to_check = get_list_of_voxels_to_check(simple_angle, d_x, d_y, start_position, next_node )
        waited_once = False #if neccesarry waiting for new local_map
        for voxel in list_to_check:

            if voxel[0] >= local_map.resolution.get('width'):
                if waited_once == False:
                    rospy.sleep(0.05)
                    waited_once = True
                voxel[0] = local_map.resolution.get('width') -1

            if voxel[1] >= local_map.resolution.get('height'):
                if waited_once == False:
                    rospy.sleep(0.05)
                    waited_once = True
                #print("voxel height outside of map")
                voxel[1] = local_map.resolution.get('height') -1

            if (local_map.map_matrix[voxel[1]][voxel[0]] >= 30):

                return False
            #that is a crash

        return True 

def distance(x_1, x_2):
    if x_1 > x_2:   #returns negative per definition
        return abs(x_1 - x_2)*(-1)

    if x_2 > x_1: #returns positiv number
        return abs(x_2 - x_1)

    if x_2 == x_1:
        return 0
        


def get_list_of_voxels_to_check(angle, d_x, d_y, start, next_node): #d_x & d_y in: +- ; angle = simple_angle ; start = [x, y] in meter

    #simple_angle is the angle defined by d_x and d_y 
    simple_angle = math.radians(angle)
    resolution = local_map.resolution.get('pixel_size')
    
    voxel_to_check= []
    multiplier_x = 1    #in positive direction
    multiplier_y = 1    #in pos direction
    if d_x < 0:
        multiplier_x = -1
    if d_y < 0:
        multiplier_y = -1
    
    i = 0
    while ( ( (abs(d_x) - abs((i*resolution*math.sin(simple_angle)))) >= 0) or ( (abs(d_y) - abs((i*resolution*math.cos(simple_angle)))) >=0) ):
        if d_x >= 3000 or d_y >= 3000:
            width = local_map.resolution.get('width')
            height = local_map.resolution.get('height')
            res = local_map.resolution.get('pixel_size')
            #before take off mavros/trajectory/desired is < -10000
            return [[width/res , height/res ]]

        x = start[0] + (d_x - multiplier_x*i*math.sin(simple_angle)*resolution)
        y = start[1] + (d_y - multiplier_y*i*math.cos(simple_angle)*resolution)

        x_ind, y_ind = get_voxel_inds(x, y)
        if (math.isnan(x_ind) == True or math.isnan(y_ind) == True ):
            i = i+1
            continue

        check_voxel = [x_ind, y_ind] # width, height = x , y
        voxel_to_check.append(check_voxel)
        i = i +1

    return voxel_to_check

def check_node(x_node, y_node):
    pixel = local_map.resolution.get('pixel_size')
    width = local_map.resolution.get('width')
    height = local_map.resolution.get('height')
    if (local_map.map_origin[0] >= x_node or local_map.map_origin[1] >= y_node):
        return False 

    else:
        if (x_node > local_map.map_origin[0]+ pixel*width):
            return False

        if (y_node > local_map.map_origin[1] + pixel*height):
            return False 
        return True

def calc_map_pixel_steps(angle, d_x, d_y):

    delta_x = 0     # moves cells each line by delta_x to a side  
    x_step = 1      #numbers of cells in line to check
    pixel_size = local_map.resolution.get('pixel_size')
    i = 1
    while i <= d_x:
        #print("x_step: " +str(i))
        if (i*pixel_size*math.tan(abs(angle)) >= pixel_size):
            x_step = i

            delta_x = i -1

            if ( delta_x == 0):

                x_step = x_step +1
                delta_height = pixel_size*math.tan(abs(angle))
                numerator = int(delta_height / pixel_size) +1

                delta_x = float(1.0/numerator)

            break
        i = i +1

    return delta_x, x_step

    

def calc_angle(start_vec, end_vec):
    if math.isnan(start_vec[0]) or math.isnan(start_vec[1]):
        return 0 ,0
    if math.isnan(end_vec[0]) or math.isnan(end_vec[1]):
        return 0 ,0

    d_x = distance(start_vec[0], end_vec[0])
    d_y = distance( start_vec[1], end_vec[1])

    if d_y != 0:
        simple_angle = abs(math.degrees(math.atan(d_x / d_y)))
    else:
        simple_angle = 0

    if (d_y >= 0):
        if (d_x > 0):
            angle =  math.degrees(math.atan(d_y/d_x))
            return simple_angle, angle
        else:
            if (d_x < 0):
                angle = (180 + math.degrees(math.atan(d_y / d_x)))
                return simple_angle, angle 
            if (d_x == 0):
                return simple_angle, 90
    else:
        #print("angle is negative")
        if d_x > 0:
            angle = -180 -  math.degrees(math.atan(d_y / d_x))
            return simple_angle, angle
        if d_x < 0:
            angle = -1*  math.degrees((math.atan(d_y / d_x)))
            return simple_angle, angle 
        if d_y == 0 and d_x >0:
            return simple_angle, 0
        if d_y == 0 and d_x < 0:
            return simple_angle, -180 

        else: #d_x == 0
            return simple_angle, -90
            

        

#returns the indexs of local_map.map_matrix[x_int][y_int] (column and height)
def get_voxel_inds(x , y):
    x_origin = local_map.map_origin[0]
    y_origin = local_map.map_origin[1]
    resolution = local_map.resolution.get('pixel_size')

    if (x >= x_origin and y >= y_origin):
        x_int = 0
        while (x_int +1 <= local_map.resolution.get('width') ):
            if (x_origin + resolution + (x_int * resolution) >= x):
                break
            x_int = x_int +1
        y_int = 0
        while (y_int <= local_map.resolution.get('height') ):

            if (y_origin + resolution + (y_int*resolution) >= y) :

                break
            y_int = y_int +1


    else:
        x_int = float('NaN')   #CAN CAUSE ERROR???
        y_int = float('NaN')   #CAN Cause ERROR???? 

    #print(x_int, y_int)
    return x_int , y_int 



###########################END CLASS#########################################


def subscribe_to_topics():
    rospy.Subscriber("/local_map", nav_msgs.msg.OccupancyGrid, local_map.local_map_splitter)
    rospy.Subscriber("/mavros/local_position/pose", geometry_msgs.msg.PoseStamped, drone_1.set_drone_position)
    rospy.Subscriber("/start_planning", RRT_Star_Call, local_map.set_target)
    #print("I did subscribe")


def publish_path(path):#, pub): #path = [[x_1, y_1], [x_2,..], ..] #first= target_position
    safe_path = Path_coll()  #new ROS message
    safe_path.stamp = rospy.get_rostime()
    pub = rospy.Publisher('safe_path', Path_coll, queue_size=10)
    #path is in opposite order: 1.node target_position
    j = len(path) - 1
    while j >= 0:
        for node in path[j]:
            safe_path.safe_path.append(node)
        j = j -1
    
    #GONE for good
    #safe_path.cost = calculate_cost(path)
    ##Always accept directr pass: cost should be lower than the true way
    #if len(path)==2:
    #    safe_path.cost = safe_path.cost -3 
    #safe_path.desired_altitude = drone_1.desired_altitude

    #path is delivered in reverse order, hence the zero
    safe_path.target_position = path[0]
    pub.publish(safe_path)


def calculate_cost(path):

    i = 0
    cost = 0
    while i <= len(path) -2:

        x_1 ,y_1 = path[i][0], path[i][1]
        x_2, y_2 = path[i+1][0], path[i +1][1]
        d_x = (x_2 - x_1) **2
        d_y = (y_2 - y_1) **2
        cost = cost + math.sqrt(d_x + d_y)
        i = i +1
    return cost

def pub_map_maximizer(i):

    pub = rospy.Publisher('/maximize_map', Maximizer, queue_size= 10)
    msg = Maximizer()

    if i <= 5:  
        msg.expand_map = i
        msg.stamp = rospy.get_rostime()
        pub.publish(msg)


if __name__ == "__main__":
    rospy.init_node('local_planner', anonymous= True)
    drone_position = [0, 0, 0, 0, 0, 0]
    drone_1 = Drone(drone_position, desired_altitude=0)
    local_map = Local_Map([0, 0], {"width": 0, "height": 0, "pixel_size": 1}, [[0][0]], [0, 0, 0], maximizer=0 )
    subscribe_to_topics()
    rospy.sleep(0.5)  #NEEDED TO GET DATA FROM Subscriber

    start_0 = [drone_1.drone_pose[0], drone_1.drone_pose[1]]
    goal_0 =  [local_map.target_position[0], local_map.target_position[1]]

    local_map_origin = local_map.map_origin
    pixel_size = local_map.resolution.get('pixel_size')
    width = local_map.resolution.get('width')
    height = local_map.resolution.get('height')

    rrt_star = RRT_Star( start_0 , goal_0, rand_area_width=[local_map.map_origin[0], local_map_origin[0]+ width*pixel_size ], rand_area_height= [local_map.map_origin[1], local_map_origin[1] + height*pixel_size ] )
    start_time = time.time()
    path = rrt_star.planning()
    pub = rospy.Publisher('safe_path', Path_coll, queue_size=10)

    if path is None:
        del rrt_star.node_list

    else:

        publish_path(path)#, pub )
        del rrt_star.node_list

    last_time_published = time.time()
    while not rospy.is_shutdown():
        if rrt_star.start_planning == True:
            try:
                print("#############")
                print("Start")
                for value in local_map.target_position:
                    if math.isnan(value) == True:
                        path = [ [ ] ]
                        #publish_path(path) #, pub)
                        pass
                start_time = time.time()
                check_also_direct_path = False
                if check_also_direct_path == True: 
                    if (rrt_star.check_2node_collision(rrt_star.start , rrt_star.goal_node) == True):
                        print("end: " +str(rrt_star.goal_node.x)+ ", " + str(rrt_star.goal_node.y))
                        path = [ [ rrt_star.goal_node.x, rrt_star.goal_node.y ], [rrt_star.start.x, rrt_star.start.y] ]
                        publish_path(path) #, pub)
                        if local_map.maximizer >= 0:
                            local_map.maximizer = 0
                            pub_map_maximizer(local_map.maximizer)
                        #rospy.sleep(0.025)  #publishes very fast(~200 per sec)
                         
                path = rrt_star.planning()
                if path is not None :
                    #PATH IS SOMETIMES JUST 2 Points without target position is path_list
                    print("safe::: " + str(path))
                    publish_path(path)#, pub)
                    last_time_published = time.time()
                    del rrt_star.node_list
                    #rospy.sleep(0.025)
                else:
                    if (time.time() - last_time_published >= 2):
                        last_time_published = time.time()              
                        if local_map.maximizer <= 3:
                            local_map.maximizer = local_map.maximizer + 1
                            pub_map_maximizer(local_map.maximizer)
                    del rrt_star.node_list
                
                print(rrt_star.start_planning)
                print("End")
                print("#############")
            except KeyboardInterrupt:
                sys.exit()
        else:
            print("I am not needed right now!")
            rospy.sleep(0.5)

        
            
        #rospy.spin()