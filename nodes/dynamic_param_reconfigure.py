#!/usr/bin/env python
import dynamic_reconfigure.client
import rospy
import roslib
import tf
import geometry_msgs

#defines the altitude for mapping the occupancy_map 
#deletes pointcloud below or above the the vehicle_altitude +- ELEVATION_SAFTEY_DISTANCE  
def set_dyn_param(msg):
    #Parameters for mapping
    ELEVATION_SAFTEY_DISTANCE = 3   # Save elevation distance in meters | Maps occupancy in vehicles altitude plus/minus this value 
    MIN_ALT_2_MAP = 1               # Below this altitude not advisable to map [Unit in: meter]
    
    vehicle_altitude =  round(msg.pose.position.z)
    pointcloud_min_z = vehicle_altitude - ELEVATION_SAFTEY_DISTANCE
    pointcloud_max_z = vehicle_altitude + ELEVATION_SAFTEY_DISTANCE
    
    if(MIN_ALT_2_MAP >= pointcloud_min_z ):
        pointcloud_min_z = MIN_ALT_2_MAP

    client = dynamic_reconfigure.client.Client("octomap_server")
    params = { 'pointcloud_min_z' : pointcloud_min_z , 'pointcloud_max_z' : pointcloud_max_z, 'occupancy_min_z' : pointcloud_min_z , 'occupancy_max_z' : pointcloud_max_z, }
    client.update_configuration(params)
    
# listens to Topic to get estimated altitude from FC 
def get_vehicle_altitude():
    #waits for single message from topic
    msg = rospy.wait_for_message("/mavros/local_position/pose", geometry_msgs.msg.PoseStamped)
    set_dyn_param(msg)


if __name__ == "__main__":
    rospy.init_node('octomap_param_reconfigurator', anonymous = True)
    
    get_vehicle_altitude()
    while not rospy.is_shutdown():
        get_vehicle_altitude()
        rospy.sleep(0.8) #waits 0.8 sec