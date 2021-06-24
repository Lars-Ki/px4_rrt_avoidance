#!/usr/bin/env python
import roslib
import rospy
#roslib.load_manifest('collsison_sim')

import tf
import geometry_msgs

def send_tf_pose(msg):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.pose.position.x, msg.pose.position.y, msg.pose.position.z),
                     (msg.pose.orientation.x, msg.pose.orientation.y , msg.pose.orientation.z, msg.pose.orientation.w),
                     rospy.Time.now(),
                     "fcu",         #child frame
                     "local_origin")           #parent Frame

def get_vehicle_pose():
    rospy.Subscriber("/mavros/local_position/pose", geometry_msgs.msg.PoseStamped , send_tf_pose)


if __name__ == '__main__':
    rospy.init_node('tf_broadcaster')
    get_vehicle_pose()
    while not rospy.is_shutdown():
        rospy.spin()
    
