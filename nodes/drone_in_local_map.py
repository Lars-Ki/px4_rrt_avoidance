#!/usr/bin/env python
import roslib
import rospy
import tf
import geometry_msgs

def get_vehicle_pose():
    rospy.Subscriber("/mavros/local_position/pose", geometry_msgs.msg.PoseStamped , send_drone_pose)


def send_drone_pose(msg):
    #new_msg = PoseStamped()
    msg.pose.position.z = 0
    pub_msg(msg)
    print(msg)
    
def pub_msg(msg):
    pub = rospy.Publisher("/drone_map", geometry_msgs.msg.PoseStamped, queue_size=10)
    pub.publish(msg)

if __name__ == '__main__':
    rospy.init_node('drone_map')
    get_vehicle_pose()
    while not rospy.is_shutdown():
        rospy.spin()