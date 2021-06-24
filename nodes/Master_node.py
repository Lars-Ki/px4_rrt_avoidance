#!/usr/bin/env python
import rospy
import mavros_msgs
from mavros_msgs.msg import Trajectory
from mavros_msgs.msg import CompanionProcessStatus
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandTOL 
from px4_rrt_avoidance.msg import Path_coll
from px4_rrt_avoidance.msg import RRT_Star_Call
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import OccupancyGrid 
from local_planner import calc_angle
from debugging_functions import show_checked_path
from debugging_functions import visualize_path
from ParameterReaderClass import ParameterViewer
import geometry_msgs
import numpy as np
import math
import copy
import time
import sys

from set_nan import set_values_to_nan




class Path:
    def __init__(self, last_wp, target_wp, incremental_wp, current_increment, LSP_OT ,empty_msg ,on_track = True, incremental_resolution = 3, acceptance_rad = 0.7 ): #WP: waypoint
        self.last_wp = last_wp                  # [x_0, y_0, z_0] last target_wp from trajectory/desired msg that is not this target_wp
        self.target_wp = target_wp              # [x_n, y_n, z_n]
        self.incremental_wp = incremental_wp    # [ [x_0, y_0, z_0], [x_1, y_1, z_1], ...., [x_n, y_n, z_n] ]   index i: 
        self._current_increment = current_increment # index i of the incrementel_wp
        self.LSP_OT = LSP_OT                    #LST_OP := Last Safe Position On Track [x_pos, y_pos]
        self.empty_msg = empty_msg              # Trajectory msg full of nan
        self.on_track = on_track                # True: RRT* not needed --- False: RRT* navigates Drone to next safe incremental_wp
        self.incremental_resolution = incremental_resolution #distance between elements in incremental_wp 
        self.acceptance_radius_wp = acceptance_rad

    def set_LSP_OT(self, WP):
        #print("SET_LSP_OT")
        #print(WP)
        self.LSP_OT = WP

    def getIncrementalResolution(self):
        return self.incremental_resolution
    
    def increase_increment(self):
        #print("increase")
        print('Mission current WP and RRT current WP')
        try:
            print(mission.incremental_wp[mission._current_increment])
            print(rrt_path.incremental_wp[rrt_path._current_increment])
        except IndexError:
            pass
        
        if len(self.incremental_wp) > self._current_increment + 1:
            self._current_increment += 1

        

    def set_safe_index(self, index):
        #print("set safe index")
        self._current_increment = index
    
    def show_current_path(self):
        #print("show_c_path")

        rest_of_path = []
        i = copy.deepcopy(self._current_increment) 
        while i < len(self.incremental_wp):
            rest_of_path.append(self.incremental_wp[i])
            i += 1
        visualize_path(rest_of_path, 'followed_path', [drone_1.position[0], drone_1.position[1] ] )

    def append_alternative(self, safe_alter_pos):
        #print("In append new Element to list")
        #print(safe_alter_pos)

        self.incremental_wp.append(safe_alter_pos)
        self._current_increment = len(self.incremental_wp) -1
        self.target_wp = safe_alter_pos


    def find_closest_alternative(self):
        last_wp = self.target_wp
        index_last_wp = get_voxel_index(last_wp)
        geo_pos = search_closest_save_point(index_last_wp)
        if len(geo_pos) == 2:
            self.append_alternative(geo_pos)
        else: 
            x_up = 1
            y_up = 1
            if index_last_wp[0] >= local_map.resolution.get('width')/2:
                x_up = -1
            if index_last_wp[1] >= local_map.resolution.get('height')/2:
                y_up = -1
            width = local_map.resolution.get('width')
            height =  local_map.resolution.get('height')
            max_iteration = 1000
            while not geo_pos :
                print("Emergency!!! Can't finde save position close enough to target position!")
                if index_last_wp[0] > 0 and index_last_wp[0] < width:
                    index_last_wp[0] += x_up
                if index_last_wp[1] > 0 and index_last_wp[0] < height:
                    index_last_wp[1] += y_up
                geo_pos = search_closest_save_point(index_last_wp)
                if max_iteration == 0:
                    break
                max_iteration -= 1
            if len(geo_pos)>=2:
                self.append_alternative(geo_pos)
        request_RRT_Star_path()


    def find_safe_increment(self, start_index= None):
        if start_index is None:
            start_index = copy.deepcopy(self._current_increment)
        
        i = start_index
        while i < len(self.incremental_wp) :
            possible_wp = self.incremental_wp[i]
            index_wp = get_voxel_index(possible_wp) # this value must be outside of the outer rim of the safetx zone -> RRT won't find path otherwise
            if local_map.getOccupancyStatus30andUP(index_wp) == True: 
                return i
            i += 1

        
        # last Wp is inside obstacle
        # RRT_Star (local_planner) is able to find a Position close by that is not occupied 
        # or inside the safty_distance radius
        #i = len(mission.incremental_wp) -1
        self.find_closest_alternative()


        return len(self.incremental_wp) -1
            #print("Last WP is inside an obstacle!!")

    def wp_increment_occupied(self):
        #print("wp_increment_occ")
        try:
            index = get_voxel_index( self.incremental_wp[self._current_increment] )
            return local_map.get_occupancy_status(index)
        except IndexError:
            if self._current_increment == 1 and len(self.incremental_wp) <= 1:
                index = get_voxel_index( self.incremental_wp[0] )
                return local_map.get_occupancy_status(index)

        # True = free
        #False = occupied


    def trajectory_Callback(self, msg):
        #print("traj_Call")
        #print(msg.point_2.position)
        #print(self.target_wp)
        desired = msg.point_2.position
        #print("desired WPs:")
        #print(desired)
        compare_vec = [desired.x, desired.y, desired.z]
        
        all_elements_same = True                        # [0,0,0]
        if desired.x != self.target_wp[0]:              #x :nan 
            all_elements_same = False                   #y nan
        if desired.y != self.target_wp[1]:              #z nan 
            all_elements_same = False
        if desired.z != self.target_wp[2]:
            all_elements_same = False 
        if math.isnan(desired.z) == True:
            all_elements_same = True

        
        #When we appended a safe geo position to incremental_wp
        # desired WP will be in list
        # easy check: length of list is longer than expected
        original_mission_path_length = calc_3D_distance(self.last_wp, self.target_wp)
        count_original_wp_list = original_mission_path_length / self.incremental_resolution

        if all_elements_same == False and len(self.incremental_wp) -1 >= count_original_wp_list:
            i = len(self.incremental_wp)-1
            while i >= 0:
                # this means our desired Wp is somewhere in the list:
                # We dont need to set the desired position as target position
                if self.incremental_wp[i] == compare_vec:
                    all_elements_same = True
                if i == 0 and self.incremental_wp[i] != compare_vec:
                    all_elements_same = False 
                i-=1 
            

        #new desired trajectory
        #set current increment to one since we are alreade at the first element of the path
        if all_elements_same == False :
            #print("new target position detected")
            self.last_wp = self.target_wp 
            self.target_wp = [desired.x, desired.y , desired.z]
            self._current_increment = 1
            self.incremental_wp = [self.last_wp]

            d_x = self.target_wp[0] - self.last_wp[0]
            d_y = self.target_wp[1] - self.last_wp[1]

            if d_x == 0:    #b does not excist 
                #print("b does not excist")
                multiplier = 1
                if d_y < 0:
                    multiplier = -1
                
                while self.calc_distance_sqr( self.incremental_wp[len(self.incremental_wp) - 1], self.target_wp) > self.incremental_resolution**2:
                    last_incremental_wp = self.incremental_wp[len(self.incremental_wp) -1]
                    next_step_y = last_incremental_wp[1] + multiplier*self.incremental_resolution                      # delta_resolution = incremental_resolution = 3 
                    self.incremental_wp.append( [self.incremental_wp[0][0], next_step_y , self.target_wp[2] ] )   # [x_1, y + delta_resolution, z_1]
                    

                self.incremental_wp.append(self.target_wp)  # last wp in path 
                    
                
            else:
                m = d_y/d_x 

                    # assume we are heading east (positive x)
                x_multiplier = 1
                if d_x < 0: # we are heading west (negative x) 
                    x_multiplier = -1

                y_multiplier = 1 # heading north (positive y axis)
                if d_y < 0:
                    y_multiplier = -1 #heading south (negative y-axis)

                # f(x_1)=y_1= m*x_1 + b --> b = y_1 - m*x_1
                b = self.target_wp[1] - m*self.target_wp[0] 

                angle = math.atan(abs(m)) 
                x_increment = self.incremental_resolution*math.cos(angle)*x_multiplier
                y_increment = self.incremental_resolution*math.sin(angle)*y_multiplier 


                while self.calc_distance_sqr( self.incremental_wp[len(self.incremental_wp) - 1], self.target_wp) > self.incremental_resolution**2:
                    #print(self.calc_distance_sqr( self.incremental_wp[len(self.incremental_wp) - 1], self.target_wp))
                    last_incremental_wp = self.incremental_wp[len(self.incremental_wp) -1]
                    self.incremental_wp.append( [last_incremental_wp[0] + x_increment, last_incremental_wp[1]+ y_increment, last_incremental_wp[2] ] )

                self.incremental_wp.append( self.target_wp )

        #print("List of WPs:")
        #print(self.incremental_wp)

    
    
    
    @staticmethod
    def calc_distance_sqr(vec_1, vec_2): 

        d_x = vec_1[0] - vec_2[0]
        d_y = vec_1[1] - vec_2[1]

        return (d_x**2 + d_y**2)    #distance squared of two vectors

class RRT_Star_Path(Path):
    def __init__(self, incremental_wp, current_increment, cost = float('inf'), number_cells_passing_safety_rim=0):
        self.incremental_wp = incremental_wp            #List of WP
        self._current_increment = current_increment      #index no
        self.cost = cost
        self.number_cells_passing_safety_rim = number_cells_passing_safety_rim


    def reset_path(self):
        del self.incremental_wp
        self.incremental_wp = [[]]
        self._current_increment = 1
        self.cost = float('inf')
        self.number_cells_passing_safety_rim = 0

    def setPassingSafetyRim(self, number_of_cells):
        self.number_cells_passing_safety_rim = number_of_cells

    def safe_waypoints(self, msg):
        #print("In safe waypoints")
        drone_1.pos_hold = False

        #print(mission.incremental_wp[mission._current_increment])
        target_vector = []
        current_target = []
        for element in msg.target_position:
            target_vector.append(element)
        try:
            target_vector.append(mission.incremental_wp[mission._current_increment][2]) #add z to compare 
            current_target = mission.incremental_wp[mission._current_increment]
        except IndexError:
            if len(mission.incremental_wp) <= 1 and mission._current_increment == 1 :
                target_vector.append(mission.incremental_wp[0][2]) 
                current_target = mission.incremental_wp[0]
            else: 
                target_vector.append(mission.incremental_wp[mission._current_increment - 1][2]) 
                current_target = mission.incremental_wp[mission._current_increment - 1]

        if (target_vector == current_target ) and drone_1.in_safety_zone == False:
            msg_list = self.save_as_list(msg.safe_path)
            
            #RVIZ visaulize suggested path
            visualize_path(msg_list, 'suggested_path', drone_1.position)
            msg_list = self.smooth_end_path(msg_list)
            cost = recalculate_cost(msg_list)

            min_acceptance_cost = 1
            if math.isinf(self.cost) == True or cost <= self.cost - min_acceptance_cost:
                self._current_increment = 1
                self.cost = cost
                self.incremental_wp = msg_list

    def delte_first_node(self):
        self.incremental_wp.pop(self._current_increment)
        new_cost = recalculate_cost(self.incremental_wp)
        self.cost = new_cost


    def GoDirectly2NextWP(self):
        drone_position = drone_1.position
        if self._current_increment + 1 <= len(self.incremental_wp) -1:
            direct_path_is_safe = True
            after_next_wp = self.incremental_wp[self._current_increment +1]
            passing_voxel_list = get_list_of_voxels_to_check( drone_position, after_next_wp)
            for passing_voxel in passing_voxel_list:
                free = local_map.get_occupancy_status(passing_voxel)
                if free == False:
                    direct_path_is_safe = False
            if direct_path_is_safe == True:
                self.delte_first_node()


    def save_as_list(self, point_list):
        #print("In save as list")
        list_e= []
        # i = 0 and i = 1 is Drone position[x,y] --> start with increment_index 1
        i = 0
        while i <= len(point_list) -1:
            #print("in 2 while")
            x_value = point_list[i]
            y_value = point_list[i+1]
            vector = [x_value, y_value, mission.incremental_wp[mission._current_increment][2] ]
            list_e.append(vector)
            i = i +2 
        return list_e

    def smooth_end_path(self, path_list):
        #print("smooth end")
        if (len(path_list) <= 1):
            return path_list
        else:
            collision_free = True
            
            while (len(path_list) > 3 and collision_free == True):
                #print("in 1 while")
                start_node = path_list[len(path_list) -3]
                end_node = path_list[len(path_list) -1]
                voxel_index_list = get_list_of_voxels_to_check(start_node,end_node)
                for cell in voxel_index_list:
                    free = local_map.get_occupancy_status(cell)
                    if free == False:
                        collision_free = False

                if collision_free == True:
                    path_list = self.delte_sec_last_node(path_list)
        
        return path_list

    def delte_sec_last_node(self, path_list):
        #print("delete sec last element")
        path_list.pop(len(path_list)-2)
        return path_list


    def check_for_collision(self):

        start = [drone_1.position[0], drone_1.position[1]]
        end = self.incremental_wp[self._current_increment]
        passing_safety_rim = 0

        remaining_path = []
        remaining_path.append(start)
        j = copy.deepcopy(self._current_increment)
        while j <= len(self.incremental_wp) -1:
            remaining_path.append(self.incremental_wp[j])
            j +=1   

        i = 0
        while i < len(remaining_path) - 1 :
            start = remaining_path[i]
            end = remaining_path[i +1]
            check_cells = get_list_of_voxels_to_check(start, end)
            for cell in check_cells:
                free = local_map.getOccupancyStatus30andUP(cell)
                if free == False:
                    just_outer_rim = local_map.get_occupancy_status(cell)
                    if just_outer_rim == True:
                        passing_safety_rim += 1
                    else:
                        return False
            i += 1
        self.setPassingSafetyRim(passing_safety_rim)
        new_cost = recalculate_cost(remaining_path)
        self.cost = new_cost
        return True

class DroneVelocity:
    def __init__(self, x_vel=0, y_vel=0, z_vel=0):
        self.x_vel = x_vel
        self.y_vel = y_vel
        self.z_vel = z_vel

    def getFullVelocity(self):
        return math.sqrt( self.x_vel**2 + self.y_vel**2 + self.z_vel**2 )

    def get2DVelocity(self):
        return math.sqrt( self.x_vel**2 + self.y_vel**2)

    def setVelocities(self, x, y, z):
        self.x_vel = x
        self.y_vel = y
        self.z_vel = z


class Drone:
    def __init__(self, position, last_safe_position, last_yaw, velocity ,max_velocity, max_acc ,pos_hold = False, in_safety_zone= False):
        self.position = position                     # [x, y, z, x, y, z, w]
        self.last_safe_position = last_safe_position # [x, y, z]
        self.last_yaw = last_yaw                     # yaw angle
        self.velocity = velocity
        self.__max_velocity_in_mission = max_velocity    # PX4 Parameter
        self.__max_acc = max_acc                        # PX4 Parameter
        self.pos_hold = pos_hold                     # emergency stop 
        self.in_safety_zone = in_safety_zone            # don't accepts rrt* msg when in safety zone for emergency exit

    def getMaxAcc(self):
        return copy.deepcopy( self.__max_acc)

    def getMaxVelocity(self):
        return copy.deepcopy(self.__max_velocity_in_mission)

    def getCurrent2D_Velocity(self):
        return self.velocity.get2DVelocity()

    def getCurrent3D_Velocity(self):
        return self.velocity.getFullVelocity()  

    def position_Callback(self, msg):
        
        position = msg.pose.position
        ori = msg.pose.orientation
        yaw = ori.z

        self.position = [ position.x, position.y, position.z, ori.x, ori.y, ori.z, ori.w ]
        self.last_yaw = yaw

    def velocity_Callback(self, msg):
        x_vel = msg.twist.linear.x
        y_vel = msg.twist.linear.y
        z_vel = msg.twist.linear.z

        self.velocity.setVelocities(x_vel, y_vel, z_vel)

    def set_safe_geo(self, save_geo):
        self.position[0] = save_geo[0]
        self.position[1] = save_geo[1]

    def set_msg_safe_position(self, msg):

        msg = self.getInitialMsgRequirements()

        msg.point_1.yaw = calc_direction(drone_1.last_safe_position)
        #used in px4 avoidance system
        msg.point_1.position.x = drone_1.last_safe_position[0]
        msg.point_1.position.y = drone_1.last_safe_position[1]
        msg.point_1.position.z = drone_1.last_safe_position[2]

        msg.point_valid = [1, 0, 0, 0, 0]

        return msg


    def checkAbortInterpolation(self):
        if mission._current_increment ==1:
            return True
        else:
            return False


    def interpolate_target_position(self, msg):

        if self.checkAbortInterpolation():
            return msg

        #always 3m in front of the drone
        wp_distance = mission.getIncrementalResolution()*2
        #wp_1 = mission.incremental_wp[mission.current_increment]
        wp_1 = mission.incremental_wp[mission._current_increment]
        wp_2 = mission.incremental_wp[mission._current_increment + 1]
        d_x = distance(wp_1[0], wp_2[0])
        d_y = distance(wp_1[1], wp_2[1])

        y_multiplier = 1
        x_multiplier = 1


        if d_x < 0:
            x_multiplier = -1
        if d_y <0:
            y_multiplier = -1

        alpha = float('NaN')
        if d_x == 0:
            alpha = math.radians(90)
            x_pos = self.position[0]
            y_pos = self.position[1] +3

            msg.point_1.position.x = x_pos
            msg.point_1.position.y = y_pos
            return msg
        else:
            alpha = abs(math.atan(d_y/d_x))

        x_n =  wp_distance*math.cos(alpha)*x_multiplier + drone_1.position[0]
        y_n =  wp_distance*math.sin(alpha)*y_multiplier + drone_1.position[1]

        
        if x_n <= wp_1[0] and x_multiplier == 1:
            x_n = wp_1[0] + 0.2

        #x_pos = wp_1[0] + (d_y/d_x)*(y_n - wp_1[1])
        x_pos = x_n 
        y_pos = wp_1[1] + (d_y/d_x)*(x_n - wp_1[0])


        msg.point_1.position.x = x_pos
        msg.point_1.position.y = y_pos
        return msg


#*************************************************************#
#               Initial MSG Requirements                      #
#*************************************************************#

    def getInitialMsgRequirements(self):
        msg = copy.deepcopy(mission.empty_msg)
        msg.point_1.type_mask = 3064
        msg.point_1.yaw_rate = -0.0
        msg.header.stamp = rospy.get_rostime()
        msg.point_valid = [1, 0, 0, 0, 0]
        msg.command = [65535, 65535, 65535, 65535, 65535]

        if mission._current_increment == len(mission.incremental_wp) -2 and mission.on_track == True:

            msg.point_2.type_mask = 3064
            msg.point_2.yaw_rate = -0.0
            
            wp_list = mission.incremental_wp
            index = mission._current_increment 

            msg.point_2.position.x = wp_list[index + 1][0]
            msg.point_2.position.y = wp_list[index + 1][1]
            msg.point_2.position.z = wp_list[index + 1][2]

            msg.point_valid = [1, 1, 0, 0, 0]

        return msg


    def requestLanding(self, next_wp):
        d_x = distance( self.position[0], next_wp[0] )
        d_y = distance( self.position[1], next_wp[1] )
        distance_LandingPosition = math.sqrt(d_x**2 + d_y**2 )
        if distance_LandingPosition <= 0.6:
            # get gloabal position for landing request 
            pos_message = rospy.wait_for_message('mavros/global_position/global', NavSatFix)
            latitude = pos_message.latitude
            longitude = pos_message.longitude
            altitude = 0
            min_pitch = math.radians(10) #10 deg
            yaw = 0

            #request Landing service
            try:
                service_call = rospy.ServiceProxy('mavros/cmd/land', CommandTOL )
                rply = service_call(min_pitch, yaw, latitude, longitude, altitude)
            except:
                print("SOMETHING WENT WRONG CALLING THE SERVICE")

#*************************************************************#
#               Set trajectory to next WP                     #
#*************************************************************#

    def setTarget2NextWP(self, msg):
        next_wp = mission.incremental_wp[mission._current_increment]
        msg.point_1.yaw = calc_direction(next_wp)
        msg.point_1.position.x = next_wp[0]
        msg.point_1.position.y = next_wp[1]
        msg.point_1.position.z = next_wp[2]

        if next_wp[2] <= 0.2:  #Altitude below 20cm
            self.requestLanding(next_wp)
        return msg

#*************************************************************#
#               Set trajectory to second next WP              #
#*************************************************************#

    def setTarget2VeryNextWP(self, msg):
        point_after = mission.incremental_wp[mission._current_increment + 1] 
        index_point_after = get_voxel_index(point_after)

        distance_after_WP = calc_3D_distance(drone_1.position , point_after)
        distance_current_WP = calc_3D_distance(drone_1.position, next_wp)

        if local_map.get_occupancy_status(index_point_after) == True and distance_after_WP >= mission.incremental_resolution -0.1 :
            msg = self.interpolate_target_position(msg)
                    
        elif local_map.get_occupancy_status(index_point_after)== False: 
            #STILL BUGGY requesting RRT Star before off track -> not correct wp send
            safe_increment_index = mission.find_safe_increment( mission._current_increment + 1 )
            request_RRT_Star_path(safe_increment_index)
        return msg



#*************************************************************#
#               Trajectory according to RRT                   #
#*************************************************************#
    def follow_RRT_path(self, msg):
        drone_1.pos_hold = False

        next_wp = rrt_path.incremental_wp[rrt_path._current_increment]
        msg.point_1.yaw = calc_direction(next_wp)
        msg.point_1.position.x = next_wp[0]
        msg.point_1.position.y = next_wp[1]
        msg.point_1.position.z = mission.incremental_wp[mission._current_increment][2]

        if len(rrt_path.incremental_wp) -1 - rrt_path._current_increment >= 2:
            msg.point_2.type_mask = 3064
            msg.point_2.position.x = rrt_path.incremental_wp[rrt_path._current_increment +1][0]
            msg.point_2.position.y = rrt_path.incremental_wp[rrt_path._current_increment +1][1]
            msg.point_2.position.z = mission.incremental_wp[mission._current_increment][2]
            msg.point_valid = [1, 1, 0, 0, 0]
        
        elif rrt_path._current_increment == 1 and len(rrt_path.incremental_wp) -1 >= 2 and rrt_path.incremental_wp[1] == mission.incremental_wp[mission._current_increment] :
            msg.point_2.type_mask = 3064
            msg.point_2.position.x = mission.incremental_wp[mission._current_increment + 1][0]
            msg.point_2.position.y = mission.incremental_wp[mission._current_increment + 1][1]
            msg.point_2.position.z = mission.incremental_wp[mission._current_increment][2]
            msg.point_valid = [1, 1, 0, 0, 0]

        return msg

#*************************************************************#
#               Choose desired Trajectory as WP               #
#*************************************************************#

    def checkAngleCondition(self, msg):
        angle_next_wp = msg.point_1.yaw 
        angle_last_wp = calc_direction(mission.incremental_wp[-1], min_distance=False)

        delta_angle = abs(angle_last_wp - angle_next_wp)

        if delta_angle == math.radians(90):
            return False

        next_wp = mission.incremental_wp[mission._current_increment]
        #last_wp = mission.last_wp

        distance_next_wp = calc_3D_distance(drone_1.position, next_wp )

        current_acc_radius = math.sin(delta_angle) * distance_next_wp           #this is r
        max_acceptance_radius = copy.deepcopy( mission.acceptance_radius_wp)    #this is R_acc
        direct_line = math.cos(delta_angle)*distance_next_wp                    # this is z

        delta_angle_max = abs(math.atan(max_acceptance_radius / direct_line ))
        safety_measure = 0.75

        if delta_angle < delta_angle_max * safety_measure:
            return True
        else:
            return False

    def checkSpeedCondition(self, max_velocity = 0, safety_distance = 7, max_deceleration = None ):
        #max_velocity = copy.deepcopy(self.__max_velocity_in_mission)
        if max_velocity == 0:
            max_velocity =  self.__max_velocity_in_mission

        if max_deceleration is None:
            max_deceleration = self.getMaxAcc() * 0.5
        cell_size = local_map.resolution.get('pixel_size')

        breaking_distance = (max_velocity**2)/(2*max_deceleration)
        breaking_distance = breaking_distance * 2 
        
        #safety_distance = 7     #additional space  in m
        no_cells_to_check = int(round(breaking_distance / cell_size)) #+ int(round(safety_distance/cell_size))

        last_wp = copy.deepcopy(mission.incremental_wp[ len(mission.incremental_wp)-1] )
        passing_cells = get_list_of_voxels_to_check(drone_1.position, last_wp)

        if len(passing_cells)-1 < no_cells_to_check:
            no_cells_to_check = len(passing_cells)-1
        i = 0
        while i <=  no_cells_to_check :            
            if local_map.getOccupancyStatus30andUP( passing_cells[i]) == False:
                return False
            i += 1
        return True

    def makeMsgLastWP(self, msg):
        target_position = mission.incremental_wp[len(mission.incremental_wp)-1]

        msg.point_1.position.x = target_position[0]
        msg.point_1.position.y = target_position[1]
        msg.point_1.position.z = target_position[2]

        return msg

    def tryDesiredTrajectory(self, msg):
        # choose this for max velocity
        # Some conditions apply:
        # - 1) Drone is not too far away from the global path 
        #       -> compares angles of:
        #           - next incremental WP
        #           - final incremental WP 
        #       -> calculates max angle difference depending on:
        #           - accaptance radius for incremental WP
        # - 2) breaking distance is shorter than free cells

        angleCondition = self.checkAngleCondition(msg)
        if angleCondition == False:
            return msg, False
        speedCondition = self.checkSpeedCondition()
        if speedCondition == True:
            return self.makeMsgLastWP( msg), True
        else:
            return msg, False
        

#*************************************************************#
#               Check Emergancy Break Condition               #
#*************************************************************#

    def EmergencyBreakNecessary(self):
        # Flase: while breaking no cell is occupied 
        # True:  some cells are occupied
        full_velocity = self.getCurrent2D_Velocity()
        if full_velocity <= drone_1.__max_velocity_in_mission / 6:
            return False
        expected_deceleration = self.getMaxAcc() / 4 
        all_cells_free_when_breaking = self.checkSpeedCondition(max_velocity=full_velocity, safety_distance=4, max_deceleration= expected_deceleration)
        if all_cells_free_when_breaking == True:
            return False 
        else:
            return True

    def setWP_BehindDrone(self, msg):
        wp_index = mission._current_increment - 1
        full_velocity = self.getCurrent2D_Velocity()
        if wp_index >= 2 and full_velocity >= self.__max_velocity_in_mission * 0.3:
            wp_index -= 1

        wp_behind_drone = mission.incremental_wp[wp_index]
        
        msg.point_1.position.x = wp_behind_drone[0]
        msg.point_1.position.y = wp_behind_drone[1]
        msg.point_1.position.z = wp_behind_drone[2]

        

        return msg


########################################################################################

    def follow_path(self):
        msg = self.getInitialMsgRequirements()
        if mission.on_track == True:
            if self.pos_hold== True:
                msg = self.set_msg_safe_position(msg)
        
            else:
                msg = self.setTarget2NextWP(msg)
                # Case: At least 2 WP more to go
                if len(mission.incremental_wp) > mission._current_increment + 2:
                    msg, lastWP_possible = self.tryDesiredTrajectory(msg)
                    if lastWP_possible == False:
                        if self.EmergencyBreakNecessary():
                            msg = self.setWP_BehindDrone(msg)
                        else:
                            msg = self.setTarget2VeryNextWP(msg)
            publish_trajectory(msg)

        elif mission.on_track== False:
            if self.pos_hold== True:
                msg = self.set_msg_safe_position(msg)
            elif len(rrt_path.incremental_wp) -1 >= rrt_path._current_increment :    
                msg = self.follow_RRT_path(msg)
            else:
                drone_1.pos_hold = True
            publish_trajectory(msg)

########################################################################################

    def set_safe_position(self):
        if drone_1.pos_hold == False:
            #print("drone pos hold = False")
            x = round(self.position[0],1)
            y = round(self.position[1],1)
            z = round(self.position[2],1)

            index = get_voxel_index( [x, y])
            if index[0] < local_map.resolution.get('width') and index[1] < local_map.resolution.get('height'):
                try:
                    if local_map.map_matrix[index[1]][index[0]] < 30:
                        self.last_safe_position = [x,y,z]
                        self.last_yaw = drone_1.position[5]
                except IndexError:
                    print("map was nut fully updated yet")



class Local_Map:
    def __init__(self, resolution, map_origin, map_matrix):
        self.resolution = resolution            # width, height, pixel_size
        self.map_origin = map_origin            # [x,y,z, x,y,z,w] x, y, z and orientation: x, y, z, w
        self.map_matrix = map_matrix            # [[-1, -1, 0, 100], [-1, -1, -1, 0], [....]...] -1: unknown; 0: not occupied; 80: safety_distance;  100: occupied

    def map_Callback(self, msg):
        pixel_size = round(msg.info.resolution, 2)
        width = msg.info.width
        height = msg.info.height
        
        origin = [msg.info.origin.position.x,
                    msg.info.origin.position.y,
                    msg.info.origin.position.z,
                    msg.info.origin.orientation.x,
                    msg.info.origin.orientation.y,
                    msg.info.origin.orientation.z,
                    msg.info.origin.orientation.w ]


        self.resolution = {"width": width, "height": height , "pixel_size": pixel_size }
        self.map_origin = origin

        map_vector = msg.data
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

    def get_occupancy_status(self, indexes):  # indexes = [x,y]
        x = indexes[0]
        y = indexes[1]

        # Use it like this:
        #map_matrix[height][width] --> map_matrix[y][x]
        try:
            if local_map.map_matrix[y][x] > 30:
                return False
            else:
                return True
        # True: Cell is not occupied 
        # False: Cell is occupied 
        
        except IndexError:  #When width and height already at correct values but matrix is not saved yet
            print("Map matrix not ready yet!")

    def getOccupancyStatus30andUP(self, indexes):
        x = indexes[0]
        y = indexes[1]

        # Use it like this:
        #map_matrix[height][width] --> map_matrix[y][x]
        try:
            if local_map.map_matrix[y][x] >= 30:
                return False
            else:
                return True
        # True: Cell is not occupied 
        # False: Cell is occupied 
        
        except IndexError:  #When width and height already at correct values but matrix is not saved yet
            print("Map matrix not ready yet!")
        
def calc_direction(target_waypoint, min_distance=True):
    start_p = [drone_1.position[0], drone_1.position[1] ]
    end_p = target_waypoint

    d_x = end_p[0] - start_p[0]
    d_y = end_p[1] - start_p[1]
    #North (=y-axis) 
    #East (=x-axis)
    # when under 0.7 m distance keep direction
    if( d_x*d_x + d_y*d_y <= 0.5) and min_distance==True:
        #print("distance under 0.7 m ")
        return drone_1.position[5]
    else:
        #print("distance above 0.7 m")
        #North-East:
        if (d_x > 0 and d_y >= 0 ):
            #print("North-East")
            rad = math.atan(d_y / d_x)
        #North-West:
        if (d_x < 0 and d_y >= 0):
            #print("North-West")
            rad = math.pi + math.atan(d_y / d_x)
        #South-West:
        if (d_y <= 0 and d_x < 0 ):
            #print("South-West")
            rad =  (math.pi - math.atan( d_y / d_x)) * (-1)
        #South-East:
        if (d_y <= 0 and d_x >0):
            #print("South-East")
            rad = ( math.atan(d_y / d_x))
        elif():
            #print ("CAUTION: flight direction calculations: UNEXPECTED OUTCOME")
            if ( d_x == 0):
                rad = 0  
        #East - West - North or South
        if (d_x == 0 ):
            #print("d_x == 0")
            #North
            if (d_x == 0 and d_y >= 0 ):
                rad = math.pi / 2
            #South
            if (d_x == 0 and d_y <= 0):
                rad = - math.pi /2
            #East
            if (d_x >= 0 and d_y == 0):
                rad = 0 
            #West 
            if (d_x <= 0 and d_y == 0 ):
                rad = math.pi / 2
            else:
                print ("CAUTION: flight direction calculations wrong")
                rad = 0

        return rad


def recalculate_cost(path, cells_passing_rim = 0):
    #Do not recalculate cost when safe path was just deleted 
    #(considers dynamic behavior of the drone)
    if drone_1.pos_hold == True:
        return float('inf')

    i = 0
    cost = 0
    if len(path) == 1:
        d_x = path[0][0] - drone_1.position[0]
        d_y = path[0][1] - drone_1.position[1]

        distance = math.sqrt(d_x**2 + d_y**2)
        if distance <= 0.5:
            return float('inf')

        else:
            return distance
        
    else:
        while i <= len(path) -2:
            #print("in while 3")
            if i == 0:
                d_x = (path[i][0] - drone_1.position[0])**2
                d_y = (path[i][1] - drone_1.position[1])**2
                cost = cost + math.sqrt(d_x + d_y)
                d_x = (path[i+1][0] - path[i][0])**2
                d_y = (path[i+1][1] - path[i +1][1])**2
                cost = cost + math.sqrt(d_x + d_y)
                i = i+1
                continue

            x_1 ,y_1 = path[i][0], path[i][1]
            x_2, y_2 = path[i+1][0], path[i +1][1]
            d_x = (x_2 - x_1) **2
            d_y = (y_2 - y_1) **2
            cost = cost + math.sqrt(d_x + d_y)
            i = i +1
        
        cost += cells_passing_rim*local_map.resolution.get('pixel_size')*0.8
        return cost


def get_voxel_index(vec): #vec = [x,y,z] in true position 
    #print(vec)

    x = vec[0]
    y = vec[1]

    map_res = local_map.resolution.get('pixel_size')
    map_width = local_map.resolution.get('width')       #width:  x-axis
    map_height = local_map.resolution.get('height')     #height: y-axis

    if x < local_map.map_origin[0] or y < local_map.map_origin[1]:
        #print(vec)
        #print("Requested position is outside of the map!")
        return float('NaN'), float('NaN') 

    else: 
        if x > local_map.map_origin[0] + map_width*map_res or y > local_map.map_origin[1] + map_height*map_res:
            #print("Requested position is outside of the map!!!")
            return float('NaN'), float('NaN')
        else:

            x_index = int((x - local_map.map_origin[0] - map_res) // map_res) + 1
            y_index = int((y - local_map.map_origin[1] - map_res) // map_res) + 1
            if x_index <0:
                x_index = 0
            if y_index <0:
                y_index = 0

            index = [x_index, y_index]
            return index



def subscripe():
    rospy.Subscriber("/mavros/local_position/pose", geometry_msgs.msg.PoseStamped, drone_1.position_Callback )
    rospy.Subscriber('/mavros/local_position/velocity_local', geometry_msgs.msg.TwistStamped, drone_1.velocity_Callback)
    rospy.Subscriber('/mavros/trajectory/desired', mavros_msgs.msg.Trajectory, mission.trajectory_Callback )
    rospy.Subscriber('/local_map', OccupancyGrid, local_map.map_Callback )
    rospy.Subscriber('/safe_path', Path_coll, rrt_path.safe_waypoints )

def publish_trajectory(msg):
    pub = rospy.Publisher("/mavros/trajectory/generated", mavros_msgs.msg.Trajectory, queue_size= 10)
    pub.publish(msg)


#when x_1 = 2 and x_2 = -2 dx = -4 not 4 
def distance(x_1, x_2):
    if x_1 > x_2:   #returns negative per definition
        return abs(x_1 - x_2)*(-1)

    if x_2 > x_1: #returns positiv number
        return abs(x_2 - x_1)

    if x_2 == x_1:
        return 0


def get_list_of_voxels_to_check(start, end): #d_x & d_y in: +- ; angle = simple_angle ; start = [x, y] in meter; end = [x,y]
    
    angle_1, angle_2 = calc_angle(start, end)
    simple_angle = math.radians(angle_1)
    
    d_x = distance(start[0], end[0])
    d_y = distance(start[1], end[1])


    resolution = local_map.resolution.get('pixel_size')
    voxel_to_check= []
    multiplier_x = 1    #in positive direction
    multiplier_y = 1    #in pos direction
    if d_x < 0:
        multiplier_x = -1
    if d_y < 0:
        multiplier_y = -1
    
    i = 0


    if(d_y ==0 and d_x != 0) :
        simple_angle = math.radians(90)




    condition_1 = (abs(d_x) - abs((i*resolution*math.cos(simple_angle))))
    condition_2 = (abs(d_y) - abs((i*resolution*math.sin(simple_angle))))
    while ( ( condition_1 >= 0) or ( condition_2 >=0) ):

    
        x = start[0] + (d_x - multiplier_x*i*math.sin(simple_angle)*resolution)
        y = start[1] + (d_y - multiplier_y*i*math.cos(simple_angle)*resolution)

        #######CAUTION !!!!!!!!!!!!!
        if (i >= 2500):
            print("SOMETHING WENT WRONG!!!!")
            sys.exit()  #PX4 will go into failsafe
        if (d_x == 0):
            break
            
        ############################


        x_ind, y_ind = get_voxel_index([x, y])
        if(x_ind == -1 or y_ind == -1):
            i = i+1
            condition_1 = (abs(d_x) - abs((i*resolution*math.sin(simple_angle))))
            condition_2 = (abs(d_y) - abs((i*resolution*math.cos(simple_angle))))
            continue

        if (math.isnan(x_ind) == True or math.isnan(y_ind) == True ):
            i = i+1
            condition_1 = (abs(d_x) - abs((i*resolution*math.sin(simple_angle))))
            condition_2 = (abs(d_y) - abs((i*resolution*math.cos(simple_angle))))
            continue

        check_voxel = [x_ind, y_ind] # width, height = x , y
        if (len(voxel_to_check) <=0):
            voxel_to_check.insert(0, check_voxel)

        elif ( voxel_to_check[len(voxel_to_check) -1] != check_voxel):
            voxel_to_check.insert(0 , check_voxel)

        i = i +1
        condition_1 = (abs(d_x) - abs((i*resolution*math.sin(simple_angle))))
        condition_2 = (abs(d_y) - abs((i*resolution*math.cos(simple_angle))))  
    return voxel_to_check


def cell_no_to_position(cell_no): #cell_no = [x,y]

    width = local_map.resolution.get('width')
    height = local_map.resolution.get('height')
    resolution = local_map.resolution.get('pixel_size')
    x_origin = local_map.map_origin[0]
    y_origin = local_map.map_origin[1]

    x_cell = cell_no[0] +1   #0...resolution is cell no 0
    y_cell = cell_no[1] +1
    
    x_pos = x_origin + x_cell*resolution + resolution/2
    y_pos = y_origin + y_cell*resolution + resolution/2
    if cell_no[0] > width or cell_no[1] > height:
        return []
    try:
        z_pos = mission.incremental_wp[mission._current_increment][2]
        return [x_pos, y_pos, z_pos]
    except IndexError:
        if mission._current_increment == 1 and len(mission.incremental_wp) <= 1:
            z_pos = mission.incremental_wp[0][2]
            return [x_pos, y_pos, z_pos]
        else:
            z_pos = mission.incremental_wp[mission._current_increment -1 ][2]
            return [x_pos, y_pos, z_pos]
    


def calc_3D_distance(vec_1, vec_2):
    d_x = vec_2[0] - vec_1[0]
    d_y = vec_2[1] - vec_1[1]
    d_z = vec_2[2] - vec_1[2]

    return math.sqrt(d_x**2 + d_y**2 + d_z**2)

def go_safe_position():
    #print("going to safe")
    msg = copy.deepcopy(mission.empty_msg)

    msg.point_1.yaw_rate = -0.0
    msg.header.stamp = rospy.get_rostime()
    msg.point_valid = [1, 0, 0, 0, 0]
    msg.command = [65535, 65535, 65535, 65535, 65535]

    msg.point_1.yaw = calc_direction(drone_1.last_safe_position)
    msg.point_1.position.x = drone_1.last_safe_position[0]
    msg.point_1.position.y = drone_1.last_safe_position[1]
    msg.point_1.position.z = drone_1.position[2]

    publish_trajectory(msg)



def request_RRT_Star_path(mission_index = None):
    pub_start_request = rospy.Publisher("/start_planning", RRT_Star_Call, queue_size=1)
    msg = RRT_Star_Call()
    msg.stamp = rospy.get_rostime()
    msg.start_planning = True

    try:
        msg.target_position = mission.incremental_wp[mission._current_increment]
        if mission_index is not None:
            #if mission._current_increment +2 <= len(mission.incremental_wp):
            #    INDEX = get_voxel_index([mission.incremental_wp[mission._current_increment]])
            
            msg.target_position = mission.incremental_wp[mission_index]
        pub_start_request.publish(msg)

    except IndexError:
        if mission._current_increment == 1:
            msg.target_position = mission.incremental_wp[0]
            pub_start_request.publish(msg)
        else:
            msg.target_position = mission.incremental_wp[mission._current_increment - 1]
            pub_start_request.publish(msg)


def request_RRT_Star_stop():
    pub_stop_request = rospy.Publisher("/start_planning", RRT_Star_Call, queue_size=1)
    msg = RRT_Star_Call()
    msg.stamp = rospy.get_rostime()
    msg.start_planning = False
    msg.target_position = [0,0,0]
    pub_stop_request.publish(msg)


#def debugging_function(matrix, index):
#    show_checked_path(local_map.map_origin, matrix, local_map.resolution, [index])
#    msg = OccupancyGrid()
#    width = local_map.resolution.get('width')
#    height = local_map.resolution.get('height')
#    msg.info.width = width
#    msg.info.height = height 
#    msg.info.origin.position.x = local_map.map_origin[0]
#    msg.info.origin.position.y = local_map.map_origin[1]
#    #pub_false_map = rospy.Publisher('/false_map',OccupancyGrid, queue_size=10  )
#
#    matrix[index[1]][index[0]] = 29
#    data = []
#    i = 0
#    while i <= height-1:
#        j =0
#        while j <= width -1:
#            data.append(matrix[i][j])
#            j += 1
#        i += 1
#    msg.data = data
#    #pub_false_map.publish(msg)


def save_position_check():
    
    index = get_voxel_index(drone_1.position)

    if local_map.get_occupancy_status(index) == False: 
        save_geo_pos =  search_closest_save_point(index)
        if save_geo_pos is not None:
            drone_1.set_safe_geo(save_geo_pos)
            if drone_1.position[2] >= 5: 
                drone_1.in_safety_zone = True
                go_there(save_geo_pos)
        else: 
            print("Cant finde safe position")
            drone_1.pos_hold = True
    else:
        drone_1.in_safety_zone = False


def search_closest_save_point(t_voxel):
    search_radius = 10 #PARAMETER number of voxels to check in radius/square
                      #should not be higher than minimum of local_map radius
    width_start = t_voxel[0] - search_radius
    height_start = t_voxel[1] - search_radius
    available_voxels = []

    i = height_start
    while (i <= height_start + (2*search_radius) -1) and i < local_map.resolution.get("height"):
        #print("in while 14")
        j = width_start
        while (j <= width_start + (2*search_radius) -1) and j < local_map.resolution.get("width"):
            #print("in while 15")
            try:
                if local_map.map_matrix[i][j] <= 0 : #there must be free space when an obstacle was detected
                    available_voxels.append([j,i]) #[width, height] 
            except IndexError:
                print('one cell outside the map')
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
        z -= 1
    
    if distance_list == []:
        return []

    min_dis = min(distance_list)
    min_index = distance_list.index(min_dis)

    #minimum distance to free space always collision free due to local_map's definition 
    save_voxel = available_voxels[min_index]

    save_geo = cell_no_to_position(save_voxel)

    return save_geo



def go_there(save_vox):
    loop_time_start = time.time()
    
    while save_vox != get_voxel_index(drone_1.position):
        msg = copy.deepcopy(mission.empty_msg)

        msg.header.stamp = rospy.get_rostime()

        msg.point_1.yaw = drone_1.last_yaw
        msg.point_1.yaw_rate = -0.0



        #used in px4 avoidance system
        msg.point_valid = [1, 0, 0, 0, 0]
        msg.command = [65535, 65535, 65535, 65535, 65535]

        msg.point_1.type_mask = 3064
        msg.point_1.position.x = drone_1.last_safe_position[0]
        msg.point_1.position.y = drone_1.last_safe_position[1]
        msg.point_1.position.z = mission.target_wp[2]
        if math.isnan(msg.point_1.position.z) == True:
            msg.point_1.position.z = 2.5
        
        x, y = get_voxel_index(drone_1.position)

        try:
            if local_map.map_matrix[y][x] == 0:
                break
            if (time.time() - loop_time_start >= 5): # 5sec to get there
                #print("in continue")
                break
            publish_trajectory(msg)
        except IndexError:
            print("Index error")

def on_track_check():
    distance3D = calc_3D_distance(drone_1.position, mission.incremental_wp[mission._current_increment])
    if mission.on_track == False:
        if distance3D <= mission.acceptance_radius_wp:
            print('reached wp on track')
            print(rrt_path.incremental_wp)
            mission.on_track = True
            rrt_path.reset_path()
            print('after reset:')
            print(rrt_path.incremental_wp)
            request_RRT_Star_stop()
            mission.increase_increment()
            #rrt_path
    
    else:
        if distance3D <= mission.acceptance_radius_wp:
            mission.increase_increment()

        

if __name__ == "__main__":
    rospy.init_node("master_node", anonymous=False)
    
    empty_trajectory_message = Trajectory() 
    empty_trajectory_message = set_values_to_nan(empty_trajectory_message)

    mission = Path([0,0,0], [0,0,-100], [[0,0,0], [0,0,4]] , 1, [0,0,0], empty_trajectory_message)
    rrt_path = RRT_Star_Path([[]], 1 )
    parameterListener = ParameterViewer()

    #service_timeout = 180 #3min
    #rospy.wait_for_service('mavros/param/get', service_timeout)

    max_misssion_speed = parameterListener.getParam('MPC_XY_CRUISE')
    max_acc_mission = parameterListener.getParam('MPC_ACC_HOR')
    rospy.loginfo('UAV velocity max: ' + str(max_misssion_speed))
    rospy.loginfo('Max. acceleration: ' + str(max_acc_mission))
    if max_misssion_speed is None:
        max_misssion_speed = 5
        rospy.logwarn('UAV velocity max: ' + str(max_misssion_speed))
    if max_acc_mission is None:
        max_acc_mission = 1
        rospy.logwarn('Max. acceleration: ' + str(max_acc_mission))


    drone_velocity =  DroneVelocity()
    drone_1 = Drone([0,0,0,0,0], [0,0, 0], 0, drone_velocity ,max_misssion_speed, max_acc_mission )



    height_map = 16
    width_map  = 16
    pixel_size = 1
    map_zero = - 8 *pixel_size
    local_map = Local_Map({"width": width_map, "height": height_map, "pixel_size": pixel_size}, [ map_zero, map_zero, 0, 0, 0, 0, 0], np.zeros((height_map, width_map)))

    subscripe()
    rospy.sleep(2)


    while not rospy.is_shutdown():
        try: 
            print('-------------------------------')
            save_position_check()
            drone_1.set_safe_position()
            on_track_check()
            
            print(mission.incremental_wp[mission._current_increment])
            try:
                print(rrt_path.incremental_wp[rrt_path._current_increment])
            except IndexError:
                print('rrt empty')

            #if we are on the path check incremental waypoints for collision
            if mission.on_track == True:
                print("On Track True")
                mission.show_current_path()

                index_wp = get_voxel_index(mission.incremental_wp[mission._current_increment])
                if local_map.getOccupancyStatus30andUP(index_wp) == False:
                    print('mission wp not free ' + str(mission.incremental_wp[mission._current_increment]) )
                    mission.on_track = False
                    rrt_path.reset_path()
                    safe_increment_index = mission.find_safe_increment()
                    mission.set_safe_index(safe_increment_index)
                    request_RRT_Star_path(safe_increment_index)

                else:
                    print('mission wp free ' + str(mission.incremental_wp[mission._current_increment]) )
                    next_wp = mission.incremental_wp[mission._current_increment]
                    list_of_voxels = get_list_of_voxels_to_check(drone_1.position, next_wp)
                    safe = True 
                    for voxel in list_of_voxels:
                        if local_map.getOccupancyStatus30andUP(voxel) == False:
                            safe = False 
                    if safe == True:
                        drone_1.follow_path()
                    
                    else:
                        safe_increment_index = mission.find_safe_increment()
                        mission.set_safe_index(safe_increment_index)
                        request_RRT_Star_path(safe_increment_index)
                        mission.on_track = False

            else:
                print('rrt')
                rrt_path.GoDirectly2NextWP()
                rrt_path.show_current_path()

                rrt_target = mission.incremental_wp[mission._current_increment]
                map_cell_index = get_voxel_index(rrt_target)
                rrt_target_free = local_map.getOccupancyStatus30andUP(map_cell_index)

                if rrt_target_free == False:
                    print('target rrt: not free')
                    drone_1.pos_hold = True
                    rrt_path.reset_path()
                    safe_increment_index = mission.find_safe_increment()
                    if safe_increment_index is not None:
                        print('safe_index: ' + str(mission.incremental_wp[safe_increment_index]))
                        mission.set_safe_index(safe_increment_index)
                        request_RRT_Star_path()


                elif not rrt_path.incremental_wp[0]: # RRT*-path is empty
                    drone_1.pos_hold = True
                    request_RRT_Star_path()

                else:
                    print('check collision rrt')
                    path_collision_free = rrt_path.check_for_collision()
                    if path_collision_free == True:
                        print('is free')
                        next_wp_dis = calc_3D_distance(drone_1.position, rrt_path.incremental_wp[rrt_path._current_increment])
                        if next_wp_dis <= mission.acceptance_radius_wp:
                            print('reached a wp')
                            rrt_path.increase_increment()
                            if len(rrt_path.incremental_wp) -1 == rrt_path._current_increment:
                                print('reached end reseting')
                                rrt_path.reset_path()
                    else:
                        print("path not free: Reset")
                        drone_1.pos_hold = True
                        rrt_path.reset_path()
                        request_RRT_Star_path()
                print('Following rrt')                        
                drone_1.follow_path()

        except KeyboardInterrupt:
            sys.exit()