#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
import tf
import math

# Offset distance (2 meters behind in x-axis of UAV0's body frame)
FOLLOW_DISTANCE = 2.0

class DroneFollower:
    def __init__(self):
        rospy.init_node('uav2_follower')

        self.leader_pose = None
        self.leader_vel = None
        self.uav2_pose = None

        rospy.Subscriber('/uav0/mavros/local_position/pose', PoseStamped, self.leader_pose_cb)
        rospy.Subscriber('/uav0/mavros/local_position/velocity_local', TwistStamped, self.leader_vel_cb)
        rospy.Subscriber('/uav2/mavros/local_position/pose', PoseStamped, self.uav2_pose_cb)

        self.follower_pub = rospy.Publisher('/uav2/mavros/setpoint_position/local', PoseStamped, queue_size=10)

        self.rate = rospy.Rate(20)  # Hz

    def leader_pose_cb(self, msg):
        self.leader_pose = msg

    def leader_vel_cb(self, msg):
        self.leader_vel = msg

    def uav2_pose_cb(self, msg):
        self.uav2_pose = msg

    def follow_leader(self):
        while not rospy.is_shutdown():
            if self.leader_pose is None:
                self.rate.sleep()
                continue

            # Calculate the target pose for follower
            target = PoseStamped()
            target.header.stamp = rospy.Time.now()
            target.header.frame_id = "map"

            # Extract yaw from quaternion
            q = self.leader_pose.pose.orientation
            euler = tf.transformations.euler_from_quaternion([q.x, q.y, q.z, q.w])
            yaw = euler[2]

            # Offset 2 meters behind along the -x direction of the leader's heading
            target.pose.position.x = self.leader_pose.pose.position.x - FOLLOW_DISTANCE * math.cos(yaw)
            target.pose.position.y = self.leader_pose.pose.position.y - FOLLOW_DISTANCE * math.sin(yaw)
            target.pose.position.z = self.leader_pose.pose.position.z  # same altitude

            # Keep orientation stable or match leader
            target.pose.orientation = self.leader_pose.pose.orientation

            self.follower_pub.publish(target)
            self.rate.sleep()


if __name__ == '__main__':
    try:
        follower = DroneFollower()
        follower.follow_leader()
    except rospy.ROSInterruptException:
        pass
