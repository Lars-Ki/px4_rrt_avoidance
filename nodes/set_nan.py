#!/usr/bin/env python
import rospy
import math
import numpy as np
from mavros_msgs.msg import Trajectory

msg = Trajectory()

def set_values_to_nan(msg):
    #######point_1#################
    msg.point_1.position.x = float('Nan')
    msg.point_1.position.y = float('Nan')
    msg.point_1.position.z = float('Nan')

    msg.point_1.velocity.x = float('Nan')
    msg.point_1.velocity.y = float('Nan')
    msg.point_1.velocity.z = float('Nan')

    msg.point_1.acceleration_or_force.x = float('Nan')
    msg.point_1.acceleration_or_force.y = float('Nan')
    msg.point_1.acceleration_or_force.z = float('Nan')

    msg.point_1.yaw = float('Nan')
    msg.point_1.yaw_rate = float('Nan')

    #######point_2#################
    msg.point_2.position.x = float('Nan')
    msg.point_2.position.y = float('Nan')
    msg.point_2.position.z = float('Nan')

    msg.point_2.velocity.x = float('Nan')
    msg.point_2.velocity.y = float('Nan')
    msg.point_2.velocity.z = float('Nan')

    msg.point_2.acceleration_or_force.x = float('Nan')
    msg.point_2.acceleration_or_force.y = float('Nan')
    msg.point_2.acceleration_or_force.z = float('Nan')

    msg.point_2.yaw = float('Nan')
    msg.point_2.yaw_rate = float('Nan')

        #######point_3#################
    msg.point_3.position.x = float('Nan')
    msg.point_3.position.y = float('Nan')
    msg.point_3.position.z = float('Nan')

    msg.point_3.velocity.x = float('Nan')
    msg.point_3.velocity.y = float('Nan')
    msg.point_3.velocity.z = float('Nan')

    msg.point_3.acceleration_or_force.x = float('Nan')
    msg.point_3.acceleration_or_force.y = float('Nan')
    msg.point_3.acceleration_or_force.z = float('Nan')

    msg.point_3.yaw = float('Nan')
    msg.point_3.yaw_rate = float('Nan')

    #######point_4#################
    msg.point_4.position.x = float('Nan')
    msg.point_4.position.y = float('Nan')
    msg.point_4.position.z = float('Nan')

    msg.point_4.velocity.x = float('Nan')
    msg.point_4.velocity.y = float('Nan')
    msg.point_4.velocity.z = float('Nan')

    msg.point_4.acceleration_or_force.x = float('Nan')
    msg.point_4.acceleration_or_force.y = float('Nan')
    msg.point_4.acceleration_or_force.z = float('Nan')

    msg.point_4.yaw = float('Nan')
    msg.point_4.yaw_rate = float('Nan')

    #######point_5#################
    msg.point_5.position.x = float('Nan')
    msg.point_5.position.y = float('Nan')
    msg.point_5.position.z = float('Nan')

    msg.point_5.velocity.x = float('Nan')
    msg.point_5.velocity.y = float('Nan')
    msg.point_5.velocity.z = float('Nan')

    msg.point_5.acceleration_or_force.x = float('Nan')
    msg.point_5.acceleration_or_force.y = float('Nan')
    msg.point_5.acceleration_or_force.z = float('Nan')

    msg.point_5.yaw = float('Nan')
    msg.point_5.yaw_rate = float('Nan')

    ##########point_valid##################
    ###########and#########################
    #########time_horizon##################
    
    msg.point_valid = [1, 0, 0, 0, 0]
    msg.time_horizon = [float('NaN'), float('NaN'), float('NaN'), float('NaN'), float('NaN')]
    ###########END#######################
    return msg


if __name__ == "__main__":

    msg = Trajectory()
    set_values_to_nan(msg)