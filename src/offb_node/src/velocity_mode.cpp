/**
 * @file offb_vel_node.cpp
 * @brief Offboard control node using MAVROS velocity commands to fly a circular path with altitude control
 */

#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <cmath>

mavros_msgs::State current_state;
geometry_msgs::PoseStamped current_pose;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_vel_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(
        "uav0/mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(
        "uav0/mavros/local_position/pose", 10, pose_cb);
    ros::Publisher vel_pub = nh.advertise<geometry_msgs::TwistStamped>(
        "uav0/mavros/setpoint_velocity/cmd_vel", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>(
        "uav0/mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(
        "uav0/mavros/set_mode");

    ros::Rate rate(20.0); // 20 Hz

    // Wait for FCU connection
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    // Dummy message to initialize stream
    geometry_msgs::TwistStamped vel_msg;
    vel_msg.twist.linear.x = 0.0;
    vel_msg.twist.linear.y = 0.0;
    vel_msg.twist.linear.z = 0.0;

    // Send a few dummy setpoints before switching to OFFBOARD
    for (int i = 100; ros::ok() && i > 0; --i) {
        vel_pub.publish(vel_msg);
        ros::spinOnce();
        rate.sleep();
    }

    // Prepare arming and mode services
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

    // Wait until armed and offboard
    while (ros::ok() && (!current_state.armed || current_state.mode != "OFFBOARD")) {
        if (current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent) {
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else if (!current_state.armed &&
                   (ros::Time::now() - last_request > ros::Duration(5.0))) {
            if (arming_client.call(arm_cmd) && arm_cmd.response.success) {
                ROS_INFO("Vehicle armed");
            }
            last_request = ros::Time::now();
        }

        vel_pub.publish(vel_msg);
        ros::spinOnce();
        rate.sleep();
    }

    // Define circular motion parameters
    double radius = 20.0;        // meters
    double target_altitude = 5.0; // meters
    double altitude_kp = 0.8;    // proportional gain
    double total_time = 60.0;    // seconds to complete one full circle
    double angular_speed = 2 * M_PI / total_time; // radians/sec
    double linear_speed = radius * angular_speed; // m/s

    ROS_INFO("Starting circular velocity control with altitude hold...");

    ros::Time start_time = ros::Time::now();
    while (ros::ok()) {
        ros::Duration elapsed = ros::Time::now() - start_time;
        double t = elapsed.toSec();

        if (t > total_time) {
            break;  // Exit after one full circle
        }

        // Compute horizontal velocities
        double angle = angular_speed * t;
        double vx = -linear_speed * sin(angle);
        double vy =  linear_speed * cos(angle);

        // Altitude control (proportional)
        double current_altitude = current_pose.pose.position.z;
        double altitude_error = target_altitude - current_altitude;
        double vz = altitude_kp * altitude_error;

        // Clamp vz to avoid aggressive climbs/descents
        if (vz > 1.0) vz = 1.0;
        if (vz < -1.0) vz = -1.0;

        // Populate and publish velocity message
        vel_msg.header.stamp = ros::Time::now();
        vel_msg.twist.linear.x = vx;
        vel_msg.twist.linear.y = vy;
        vel_msg.twist.linear.z = vz;

        vel_pub.publish(vel_msg);
        ros::spinOnce();
        rate.sleep();
    }

    // Stop the drone (hover)
    vel_msg.twist.linear.x = 0.0;
    vel_msg.twist.linear.y = 0.0;
    vel_msg.twist.linear.z = 0.0;

    for (int i = 0; i < 50 && ros::ok(); ++i) {
        vel_pub.publish(vel_msg);
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("Circular path with altitude control completed.");

    return 0;
}
