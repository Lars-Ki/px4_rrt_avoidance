#!/usr/bin/env python
import rospy
import roslib
import nav_msgs
import math
import time
import sys
import geometry_msgs
from nav_msgs.msg import OccupancyGrid 
import numpy as np
from std_msgs.msg import Float64
from mavros_msgs.msg import PositionTarget
from mavros_msgs.msg import Trajectory
from mavros_msgs.msg import State
from px4_rrt_avoidance.msg import Maximizer
from px4_rrt_avoidance.msg import DynamicObstacle
import mavros_msgs
from set_nan import set_values_to_nan

class DynObsticle():
    def __init__(self, position = [None, None, None], speed = 0):
        self.position = position

    def setObtsaclePosition(self, msg):
        data_pos = msg.position
        self.position = [data_pos.x, data_pos.y, data_pos.z]

    def getPositionOfDynObstacle(self):
        return self.position

class TimeWatch():
    def __init__(self, start_time = time.time() , end_time= time.time() ):
        self.start_time = start_time
        self.end_time = end_time

    def pub_time(self, time_needed):

        pub = rospy.Publisher('/average_time_calc', Float64, queue_size=10)
        pub.publish(time_needed)

class Obstacles():
    def __init__(self, obstacle_pos_list):
        self.obstacle_pos_list = obstacle_pos_list


####  CLASS MAP ######################################
class Maps():
    def __init__(self, name, resolution, map_origin,  Map_matrix, target_pose, minimum_extension, min_edge_distance):
        self.name = name                        # global_map or local_map
        self.resolution = resolution            # width, height, pixel_size
        self.map_origin = map_origin            # [x,y,z, x,y,z,w] position: x, y, z and orientation: x, y, z, w
        self.Map_matrix = Map_matrix            # [[-1, -1, 0, 100], [-1, -1, -1, 0], [....]...] -1: unknown; 0: not occupied; 80: safety_distance;  100: occupied
        self.target_pose = target_pose          # [x, y, z]
        self.min_extension = minimum_extension  # in meters
        self.min_edge_distance = min_edge_distance                  # minimum distance to edge 

    def set_map_origin(self, map_origin):
        self.map_origin = [round(map_origin.position.x ,2), round(map_origin.position.y, 2), round(map_origin.position.z, 2), map_origin.orientation.x,  map_origin.orientation.y,  map_origin.orientation.z,  map_origin.orientation.w]
 

    def calcObstaclePosition(self, matrix_width_pos, matrix_height_pos):

        map_origin = self.map_origin

        resolution = self.resolution.get('pixel_size')
        x_pos = map_origin[0] + (resolution*(matrix_width_pos)) + resolution/2  # for center of cell
        y_pos = map_origin[1] + (resolution*(matrix_height_pos)) + resolution/2

        return [x_pos, y_pos]

    def getOccupiedPositions(self, matrix, start_matrix_pos, end_matrix_pos):

        obstacle_pos_list = []
        # matrix is still just a single vector 
        # calculate the start_positions inside the vector

        # the length of values in a row
        length_of_width_to_inspect = end_matrix_pos[0] - start_matrix_pos[0]

        # number of steps to repeat this procedure
        #print(start_matrix_pos)
        #print(end_matrix_pos)
        no_count = end_matrix_pos[1] - start_matrix_pos[1] 

        i = 0
        while i <= no_count:
            px_width = (self.resolution.get("width") )
            start_position_in_vec = start_matrix_pos[0] + i*(px_width) + start_matrix_pos[1]*px_width
            
            j = 0
            while j <= length_of_width_to_inspect :

                if matrix[start_position_in_vec + j] == 100: 

                    width_position = ( (start_matrix_pos[0] + j) ) 
                    obs_pos = self.calcObstaclePosition(  width_position  , i + start_matrix_pos[1] )
                    obstacle_pos_list.append(obs_pos)
                
                j += 1
            i += 1
        return obstacle_pos_list        


    def getUpperCorner(self):
        resolution = self.resolution.get('pixel_size')
        origin = self.map_origin
        width = self.resolution.get('width')
        height = self.resolution.get('height')
        x_upper = origin[0] + resolution*width
        y_upper = origin[1] + resolution*height
        upper_corner = [x_upper, y_upper ]
        return upper_corner 

    def getOverlappingStart(self, global_LLC, global_URC, local_LLC, local_URC ):
        width_start = None 
        height_start = None 
        #------------------------------------
        #check width start
        #case 1 local map is left to projected_map but some part is still overlapping
        if local_LLC[0] <= global_LLC[0]: 
            if local_URC[0] <= global_URC[0]:
                width_start = 0
            if local_URC[0] > global_URC[0]:
                width_start = 0
            else:
                #map outside of projected map
                pass

        else:
            # case 2 local_map starts in projected_map 
            if global_LLC[0] < local_LLC[0] and local_LLC[0] < global_URC[0]:
                delta_width = local_LLC[0] - global_LLC[0] 
                width_start = delta_width / self.resolution.get('pixel_size')

            # case 3
            else: 
                return width_start, height_start
        #------------------------------------
        
        # check height_start
        # case 1: map below projected map but some part still inside projected_map
        if local_LLC[1] < global_LLC[1]:

            if local_URC[1] < global_URC[1]:
                height_start = 0
            #case: projected map smaller than the local_map
            if local_URC[1] > global_LLC[1]:
                height_start = 0
            else:
                #when error accurres
                print("local_LLC[1]: " + str(local_LLC[1]))
                print("global_LLC[1]: " + str(global_LLC[1]))
                print("still None")
                #sys.shutdown()

        if local_LLC[1] == global_LLC[1]:
            height_start = 0
        
        else:

            #case 2: loal_map starts inside projected_map:
            if global_LLC[1] < local_LLC[1] and local_LLC[1] < global_URC[1]:
                delta_height = local_LLC[1] - global_LLC[1]
                height_start = delta_height / self.resolution.get('pixel_size')

            if local_LLC[1] > global_LLC[1]:
                delta_height = local_LLC[1] - global_LLC[1]
                height_start = delta_height / self.resolution.get('pixel_size') 

            else:
                #when error accurres
                print("local_LLC[1]: " + str(local_LLC[1]))
                print("global_LLC[1]: " + str(global_LLC[1]))
                #sys.shutdown()

        try:
            width_start = int(width_start)
            height_start = int(height_start)
        
        except TypeError:
            # values are None so do nocthing with them
            pass
        
        return width_start, height_start


    def getOverlappingEnd(self, global_LLC, global_URC, local_LLC, local_URC ):
        end_width = None 
        end_height = None 

        if global_LLC[0] < local_URC[0] and local_URC[0] < global_URC[0]:
            delta_width = local_URC[0] - global_LLC[0]
            end_width = (delta_width / self.resolution.get('pixel_size')) -1

        else:  
            end_width = global_map.resolution.get('width') -1

        if global_LLC[1] < local_URC[1] and local_URC[1] < global_URC[1]:
            delta_height = local_URC[1] - global_LLC[1]
            end_height = (delta_height / self.resolution.get('pixel_size')) -1
        
        else: 
            end_height = global_map.resolution.get('height') -1

        return int(end_width), int(end_height)

        
    def setupUnoccupiedMatrix(self):
        self.Map_matrix = np.zeros(( self.resolution.get('height') , self.resolution.get('width') ))

    def insertOccupiedPositions(self, obstacle_pos_list):
        #obstacle_pos_list = [ [x_1, y_1], [x_2, ..], ...  ]
        for position in obstacle_pos_list:
            matrix_cell = self.getMatrixPosition(position)
            #if matrix_cell[0] is not None:
            try:
                self.Map_matrix[matrix_cell[1]][matrix_cell[0]] = 100
                self.surroundSaftyDistance(matrix_cell)
            except IndexError:
                pass
        
        dyn_obstacle_pos = moving_obstacle.getPositionOfDynObstacle()
        try:
            if abs( dyn_obstacle_pos[2] - drone_1.drone_pose[2]) <= 5:
                matrix_cell = self.getMatrixPosition(dyn_obstacle_pos)
                try:
                    self.Map_matrix[matrix_cell[1]][matrix_cell[0]] = 100
                    self.surroundSaftyDistance(matrix_cell)
                except IndexError:
                    print("Obstacle not inside local_map")
            #else:
            #    pass
        except TypeError:
            pass
            

    def surroundSaftyDistance(self, matrix_cell):
        safty_distance_radius = 3
        no_cells = (safty_distance_radius / self.resolution.get('pixel_size')) 
        no_cells = int(no_cells)  #make it an integer

        i = 0 # start for surrounding cells
        safty_radius = no_cells 

        #when safty_radius starts outside of the map
        x_jump = 0
        y_jump = 0

        start_width = matrix_cell[0] - safty_radius
        start_height = matrix_cell[1] - safty_radius

        end_width = start_width + 2*safty_radius
        end_height = start_height+ 2*safty_radius
        if start_width < 0:
            x_jump = abs(start_width)
            start_width = 0
        
        if start_height < 0:
            y_jump = abs(start_height)
            start_height = 0
            no_cells = no_cells - y_jump

        #start with height
        map_max_width = self.resolution.get('width') -1
        map_max_height = self.resolution.get('height') -1


        i = 0 # start for surrounding cells
        while i + start_height <= end_height and start_height + i <= map_max_height:
            j = 0 #now width
            while j + start_width <= end_width and start_width + j <= map_max_width:

                if self.Map_matrix[start_height + i ][start_width + j] != 100:

                    # occupancy is 30 for outer rim
                    if(  (i == 0 and y_jump == 0) or i == matrix_cell[0] + no_cells ):

                        if self.Map_matrix[start_height + i ][start_width + j] < 80 :
                            self.Map_matrix[start_height + i ][start_width + j] = 30

                    elif( (j == 0 and x_jump ==0) or start_width + j ==  end_width ):

                        if self.Map_matrix[start_height + i ][start_width + j] < 80:
                            self.Map_matrix[start_height + i ][start_width + j] = 30

                    elif( start_height +i == end_height):
                        if self.Map_matrix[start_height + i ][start_width + j] < 80 :
                            self.Map_matrix[start_height + i ][start_width + j] = 30

                    else:
                        self.Map_matrix[start_height + i ][start_width + j] = 80
                j+=1
            i +=1 


    def getMatrixPosition(self, position):

        origin = local_map.map_origin
        resolution = local_map.resolution.get('pixel_size')

        x_pos = position[0]
        y_pos = position[1]

        delta_x = x_pos - origin[0]
        delty_y = y_pos - origin[1]

        width_cell = int(delta_x / resolution)
        height_cell = int(delty_y/ resolution)

        if width_cell < 0 or height_cell < 0:
            return []  

        return [width_cell, height_cell]


    def occupyMatrix(self, obstacle_pos_list):
        self.setupUnoccupiedMatrix()
        self.insertOccupiedPositions(obstacle_pos_list)


    def calcSizeOfMap(self):
        drone_pose_int = [round(drone_1.drone_pose[0]), round(drone_1.drone_pose[1]) ]
        
        lower_left_corner= [-4, -4]
        upper_right_corner= [4 , 4]

        distance_x = abs(drone_1.drone_pose[0] - global_map.target_pose[0])
        
        if (distance_x <= 2): #3m min 
            #print("distance x < 2m")
            lower_left_corner[0] = int(round(global_map.target_pose[0],0)) - local_map.min_extension
            upper_right_corner[0] = int(round(global_map.target_pose[0],0)) + local_map.min_extension
            local_map_width = ((upper_right_corner[0] - lower_left_corner[0]) / global_map.resolution.get('pixel_size'))

        else:  
            #print("distance x > 2m")
            if (drone_pose_int[0] < round(global_map.target_pose[0],0) ):
                lower_left_corner[0] = drone_pose_int[0] - local_map.min_extension
                upper_right_corner[0] = round(global_map.target_pose[0],0) + local_map.min_extension
                local_map_width = ((upper_right_corner[0] - lower_left_corner[0]) / global_map.resolution.get('pixel_size'))
            else:
                lower_left_corner[0] = round(global_map.target_pose[0]) - local_map.min_extension
                upper_right_corner[0] = drone_pose_int[0] + local_map.min_extension
                local_map_width = ((upper_right_corner[0] - lower_left_corner[0]) / global_map.resolution.get('pixel_size'))
        
        distance_y = abs(drone_1.drone_pose[1] - global_map.target_pose[1])
        
        if (distance_y <= 2):
            #print("distance y < 2m")
            lower_left_corner[1] = int(round(global_map.target_pose[1])) - local_map.min_extension
            upper_right_corner[1] = int(round(global_map.target_pose[1])) + local_map.min_extension
            local_map_height = ((upper_right_corner[1] - lower_left_corner[1])/ global_map.resolution.get('pixel_size'))
            #print("calculated height: " +str(local_map_height))

        else:
            #print("distance y > 2m")
            if (drone_pose_int[1] < global_map.target_pose[1]):
                lower_left_corner[1] = drone_pose_int[1] - local_map.min_extension
                upper_right_corner[1] = round(global_map.target_pose[1],0) + local_map.min_extension
                local_map_height = ((upper_right_corner[1] - lower_left_corner[1])/ global_map.resolution.get('pixel_size'))

            else:
                lower_left_corner[1] = round(global_map.target_pose[1]) - local_map.min_extension
                upper_right_corner[1] = drone_pose_int[1] + local_map.min_extension
                local_map_height = ((upper_right_corner[1] - lower_left_corner[1])/ global_map.resolution.get('pixel_size'))
        local_map_height = int( abs(local_map_height) )
        local_map_width = int( abs(local_map_width) )

        self.resolution['width'] = local_map_width
        self.resolution['height'] = local_map_height

        self.map_origin = [ lower_left_corner[0], lower_left_corner[1], 0, 0, 0, 0, 0 ]

        
    #arranges [subscribed] matrix into orderly manner -> style:
    #[ [n_11, n_12, n_13, ....., n_1_width],                                     
    #[n_21, .....],
    #[n_31, .....]
    #[...],
    #...
    #...
    #...
    #[n_1,height, ......, n_height,width] ]    
    def set_occupancy_matrix(self, Map_matrix):
        local_map.calcSizeOfMap()

        height = global_map.resolution.get('height') # Hoch
        width = global_map.resolution.get('width')   # Breit

        glob_map_lower_l_corner = [self.map_origin[0], self.map_origin[1]]
        glob_map_upper_r_corner = self.getUpperCorner()

        local_lower_l_corner = [local_map.map_origin[0], local_map.map_origin[1] ]
        local_upper_r_corner = local_map.getUpperCorner()

        width_start, height_start = self.getOverlappingStart(glob_map_lower_l_corner, glob_map_upper_r_corner, local_lower_l_corner, local_upper_r_corner)
        start_at_matrix_cells = [width_start, height_start]
        end_at_matrix_cells = []



        if width_start is None and height_start is None:
            # this means end of width and height are also none .... do nothing here
            pass

        else:
            width_end, height_end = self.getOverlappingEnd(glob_map_lower_l_corner, glob_map_upper_r_corner, local_lower_l_corner, local_upper_r_corner)
            end_at_matrix_cells = [width_end, height_end]
            #print("End width and height:")
            #print(end_at_matrix_cells)
        
            obstacle_pos_list = self.getOccupiedPositions(Map_matrix, start_at_matrix_cells, end_at_matrix_cells)
            obstacle.obstacle_pos_list = obstacle_pos_list
        local_map.occupyMatrix(obstacle.obstacle_pos_list)


        timer.start_time = time.time()
        self.Map_matrix = np.zeros((height, width))  # Zeile x Spalte #ACHTUNG!!!!!!!!!!!!!!!!!!!!!!!!
        j = 0 #height of map 
        i = 0  #width of map 
        k = 0 # position in Map_matrix
        while (j <= height-1):
            while (i <= width -1 ): 
                if (k <= len(Map_matrix) - 1):

                    self.Map_matrix[j][i] = Map_matrix[k]
                    k = k +1
                i = i +1 
            
            if (i == width):
                i = 0 
            j = j +1 

        local_map.publish_map()

    
    def set_target_pose(self, target_pose):
        self.target_pose = target_pose


    def publish_map(self):
        msg = OccupancyGrid()
        msg.header.stamp = rospy.get_rostime()
        msg.header.frame_id = "map"

        resolution = global_map.resolution.get('pixel_size')
        msg.info.resolution = resolution

        width = local_map.resolution.get('width')
        height = local_map.resolution.get('height')

        msg.info.width = width
        msg.info.height = height

        msg.info.origin.position.x = local_map.map_origin[0]
        msg.info.origin.position.y = local_map.map_origin[1]
        msg.info.origin.orientation.x = local_map.map_origin[2]
        msg.info.origin.orientation.y = local_map.map_origin[3]
        msg.info.origin.orientation.z = local_map.map_origin[4]
        msg.info.origin.orientation.w = local_map.map_origin[5]

        msg.data = self.matrix2Tuple()

        pub = rospy.Publisher('local_map', OccupancyGrid, queue_size=10)
        try:
            pub.publish(msg)
            timer.end_time = time.time()
            avg_time = timer.end_time - timer.start_time 
            #print("Timer: " + str( timer.end_time - timer.start_time ))
            timer.pub_time(avg_time)

        except KeyboardInterrupt:
            rospy.signal_shutdown()


    def matrix2Tuple(self):
        n = 0 #height 
        #matrix to vec
        pub_vec = []
        map_local = local_map.Map_matrix

        height = local_map.resolution.get('height')
        width = local_map.resolution.get('width')

        while n <= height -1 :
            k = 0 #width
            while k <= width -1 :
                pub_vec.append(map_local[n][k])
                #print(map_local[n][k])
                k = k +1 
            n = n +1 

        return tuple(pub_vec)
##############CLASS MAP END ###################################################


############CLASS Drone #########################
class Drone():
    def __init__(self, drone_pose, status, armed):
        self.drone_pose = drone_pose
        self.status = status
        self.armed = armed

    def set_drone_pose(self, msg):
        x = msg.pose.position.x 
        y = msg.pose.position.y
        z = msg.pose.position.z
        pitch = msg.pose.orientation.x
        yaw = msg.pose.orientation.y
        roll = msg.pose.orientation.z
        self.drone_pose = [x, y, z, roll, pitch, yaw]
#############END CLASS Drone



##############CLASS WAYPOINTS inherited by MAP and drone #################################
#Requirements:
# get Trajectory() message and set first point to vehicle position (if not flying)
# set everything else to nan 
# if flying: the first three points (point_2 to point_3) are equal 
# see surface book OneNote for further information 

class Path(Maps):
    def __init__(self, save_waypoints, empty_path ,generated_path ):
        self.save_waypoints = save_waypoints
        self.empty_path = empty_path
        self.generated_path = generated_path

##############END CLASS WAYPOINT ##############################################

def separate_msg(msg):
    resolution = {"width": msg.info.width, "height": msg.info.height, "pixel_size": round(msg.info.resolution, 2) }
    global_map.resolution = resolution 
    global_map.set_map_origin(msg.info.origin)
    global_map.set_occupancy_matrix(msg.data)



def define_target_position(msg):
    position = msg.point_2.position 
    if (math.isnan( msg.point_2.position.x ) == False):
        x_p = position.x
    if (math.isnan( msg.point_2.position.y ) == False):
        y_p = position.y
    if ( math.isnan( msg.point_2.position.z ) == False):
        z_p = position.z


    #at point 2 position z != nan
    if (math.isnan( msg.point_2.position.z ) == True and math.isnan( msg.point_2.position.z) == False): # and (math.isnan(msg.point_2.position.y) == False or math.isnan(msg.point_2.position.x) == False ) ):
        if (check_flight_state() == True):
            z_p = round(msg.point_2.position.z,0)


    if (math.isnan(msg.point_1.position.z) == True and math.isnan(msg.point_1.position.y) == True and math.isnan(msg.point_1.position.x) == True ):
        #print("All values are NaN")
        x_p = drone_1.drone_pose[0]
        y_p = drone_1.drone_pose[0]
        z_p = drone_1.drone_pose[0]
        

    try:
        target_vector = [x_p, y_p, z_p ]
        #for now keep this Info: 
        i = 0
        while (i <= len(target_vector) - 1):
            if (math.isnan(target_vector[i]) == True):
                print("Error accured: Target position is 'nan' instead of float ")
            i = i +1

        local_map.set_target_pose(target_vector)
        global_map.set_target_pose(target_vector)
    
    except UnboundLocalError:
        pass


#check if landing is active:
def check_flight_state():
    if (drone_1.status == 'AUTO.RTL' or drone_1.status == 'AUTO.LAND'):
        return True
    else:
        return False

def calc_direction():
    start_p = [drone_1.drone_pose[0], drone_1.drone_pose[1] ]
    end_p = [local_map.target_pose[0], local_map.target_pose[1]]

    d_x = end_p[0] - start_p[0]
    d_y = end_p[1] - start_p[1]

    #North (=y-achsis) 
    #East (=x-achsis)
    # when under 0.7 m distance keep direction
    if( d_x*d_x + d_y*d_y <= 0.5):
        #print("distance under 0.7 m ")
        return drone_1.drone_pose[5]
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
                #print ("CAUTION: flight direction calculations wrong")
                rad = 0

        return rad 


def flight_status(status):
    #print(status.mode)
    drone_1.status = status.mode
    drone_1.armed = status.armed

def expand_map(msg):
    multiplier = msg.expand_map
    local_map.min_extension = local_map.min_edge_distance + (multiplier * local_map.min_edge_distance)


def subscribe_to_topics():
    rospy.Subscriber('/mavros/state', mavros_msgs.msg.State, flight_status )
    rospy.Subscriber('/mavros/trajectory/desired', mavros_msgs.msg.Trajectory, define_target_position)
    rospy.Subscriber('/projected_map', nav_msgs.msg.OccupancyGrid, separate_msg)
    rospy.Subscriber('/mavros/local_position/pose', geometry_msgs.msg.PoseStamped, drone_1.set_drone_pose)
    rospy.Subscriber('/maximize_map', Maximizer, expand_map  )
    rospy.Subscriber('/dynamic_obstacle', DynamicObstacle, moving_obstacle.setObtsaclePosition )

if __name__ == "__main__":
    rospy.init_node('local_map_publisher', anonymous=False)

    #initiates different maps
    Map_matrix = np.full((8,8), -1, dtype=int)                                            # occupancy_map
    resolution = {"width": 8, "height": 8, "pixel_size": 0.50}
    map_origin = [-4 , -4, 0, 0, 0, 0]                                #x, y, z and orientation: x, y, z, w 
    drone_pose = [0, 0, 0]                                          #x and y position (2D position)
    target_pose = [0, 0, 0]                                         #Position in [x, y, z]

    empty_trajectory = Trajectory()
    empty_trajectory = set_values_to_nan(empty_trajectory)

    save_waypoints = [[0,0], [0,0] ]
    generated_way = Path(save_waypoints, empty_trajectory, empty_trajectory)    #it is pretty annoying to save the Trajectory(all_values= NaN) this way ......
    global_minimum_extension = 0
    local_minimum_extension = 6
    drone_1 = Drone(drone_pose, status = 'AUTO.LOITER', armed= False )
    global_map = Maps("global_map", resolution, map_origin, Map_matrix, target_pose, global_minimum_extension, min_edge_distance=0 )
    local_map =  Maps("local_map", resolution, map_origin,  Map_matrix, target_pose, local_minimum_extension, min_edge_distance= 6) #Bug min_edge_distance and local_minimum_extension must be same 
    
    obstacle = Obstacles([])
    moving_obstacle = DynObsticle()

    timer = TimeWatch()

    subscribe_to_topics()

    
    while not rospy.is_shutdown():
#        local_map.publish_map()

        rospy.spin()