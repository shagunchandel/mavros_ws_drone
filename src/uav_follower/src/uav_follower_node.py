#!/usr/bin/env python

import rospy
import math
import tf
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode

# Offset distance (2 meters behind in x-axis of UAV0's body frame)
FOLLOW_DISTANCE = 2.0

class DroneFollower:
    def __init__(self):
        rospy.init_node('uav2_follower')

        self.leader_pose = None
        self.leader_vel = None
        self.uav2_pose = None
        self.current_state = State()

        # Subscribers
        rospy.Subscriber('/uav0/mavros/local_position/pose', PoseStamped, self.leader_pose_cb)
        rospy.Subscriber('/uav0/mavros/local_position/velocity_local', TwistStamped, self.leader_vel_cb)
        rospy.Subscriber('/uav2/mavros/local_position/pose', PoseStamped, self.uav2_pose_cb)
        rospy.Subscriber('/uav2/mavros/state', State, self.state_cb)

        # Publisher
        self.follower_pub = rospy.Publisher('/uav2/mavros/setpoint_position/local', PoseStamped, queue_size=10)

        # Services
        rospy.wait_for_service('/uav2/mavros/cmd/arming')
        rospy.wait_for_service('/uav2/mavros/set_mode')
        self.arming_client = rospy.ServiceProxy('/uav2/mavros/cmd/arming', CommandBool)
        self.set_mode_client = rospy.ServiceProxy('/uav2/mavros/set_mode', SetMode)

        self.rate = rospy.Rate(20)  # Hz

    def leader_pose_cb(self, msg):
        self.leader_pose = msg

    def leader_vel_cb(self, msg):
        self.leader_vel = msg

    def uav2_pose_cb(self, msg):
        self.uav2_pose = msg

    def state_cb(self, msg):
        self.current_state = msg

    def send_initial_setpoints(self):
        # Send a few dummy setpoints before switching to OFFBOARD
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = 2  # takeoff height
        pose.pose.orientation.w = 1.0

        for _ in range(100):  # About 5 seconds of data @ 20Hz
            self.follower_pub.publish(pose)
            self.rate.sleep()

    def arm_and_offboard(self):
        last_request = rospy.Time.now()
        while not rospy.is_shutdown():
            if self.current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_request > rospy.Duration(5.0)):
                mode_resp = self.set_mode_client(custom_mode="OFFBOARD")
                if mode_resp.mode_sent:
                    rospy.loginfo("OFFBOARD mode enabled")
                last_request = rospy.Time.now()

            if not self.current_state.armed and (rospy.Time.now() - last_request > rospy.Duration(5.0)):
                arm_resp = self.arming_client(True)
                if arm_resp.success:
                    rospy.loginfo("Vehicle armed")
                last_request = rospy.Time.now()

            if self.current_state.mode == "OFFBOARD" and self.current_state.armed:
                rospy.loginfo("Drone is in OFFBOARD mode and armed.")
                break

            self.follower_pub.publish(PoseStamped())  # Keep publishing to maintain OFFBOARD
            self.rate.sleep()

    def follow_leader(self):
        self.send_initial_setpoints()
        self.arm_and_offboard()

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

            # Match leader's orientation
            target.pose.orientation = self.leader_pose.pose.orientation

            self.follower_pub.publish(target)
            self.rate.sleep()


if __name__ == '__main__':
    try:
        follower = DroneFollower()
        follower.follow_leader()
    except rospy.ROSInterruptException:
        pass
