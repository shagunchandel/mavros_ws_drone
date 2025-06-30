#!/usr/bin/env python3
import rospy
import math
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

def circle_mission(radius=20.0, total_time=60.0, altitude=10.0):
    rospy.init_node('circle_mission_node', anonymous=True)

    pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=10)
    rate_hz = 20  # publish rate (Hz)
    rate = rospy.Rate(rate_hz)

    start_time = rospy.Time.now()
    rospy.loginfo("Starting circle mission")

    # We'll assume the center of the circle is (0, 0) local frame for simplicity.
    # You can update this to the drone's actual current position by subscribing to /mavros/local_position/pose

    while not rospy.is_shutdown():
        elapsed = (rospy.Time.now() - start_time).to_sec()
        if elapsed > total_time:
            rospy.loginfo("Completed the circle mission")
            break

        # Angle in radians, complete 2*pi in total_time seconds
        angle = 2 * math.pi * (elapsed / total_time)

        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        z = altitude

        pose = PoseStamped()
        pose.header = Header()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = "map"

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z

        # Orientation fixed (can be set if you want drone yaw to face tangent direction)
        pose.pose.orientation.x = 0
        pose.pose.orientation.y = 0
        pose.pose.orientation.z = 0
        pose.pose.orientation.w = 1

        pub.publish(pose)
        rate.sleep()

if __name__ == '__main__':
    try:
        circle_mission(radius=20, total_time=60, altitude=10)
    except rospy.ROSInterruptException:
        pass
