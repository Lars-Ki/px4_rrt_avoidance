#!/usr/bin/env python
import roslib
import rospy
#roslib.load_manifest('collsison_sim')

import tf
import geometry_msgs
from nav_msgs.msg import Odometry

def send_tf_pose(msg):
    print(msg.pose.pose.position)

    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z),
                     (msg.pose.pose.orientation.x, msg.pose.pose.orientation.y , msg.pose.pose.orientation.z, msg.pose.pose.orientation.w),
                     rospy.Time.now(),
                     "drone",         #child frame
                     "map")           #parent Frame

def get_vehicle_pose():
    rospy.Subscriber("/gazebo/true_pose", Odometry , send_tf_pose)


if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')
    get_vehicle_pose()
    while not rospy.is_shutdown():
        rospy.spin()
    
