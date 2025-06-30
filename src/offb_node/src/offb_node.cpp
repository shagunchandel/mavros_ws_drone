/**
 * @file offb_node.cpp
 * @brief Offboard control node for following a circular path using MAVROS
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <vector>
#include <cmath>

mavros_msgs::State current_state;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

// Linear interpolation between two points
geometry_msgs::Point interpolate(const geometry_msgs::Point& start, const geometry_msgs::Point& end, double t){
    geometry_msgs::Point p;
    p.x = start.x + (end.x - start.x) * t;
    p.y = start.y + (end.y - start.y) * t;
    p.z = start.z + (end.z - start.z) * t;
    return p;
}

// Helper to create points
geometry_msgs::Point makePoint(double x, double y, double z) {
    geometry_msgs::Point p;
    p.x = x; p.y = y; p.z = z;
    return p;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>(
        "mavros/state", 10, state_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>(
        "mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>(
        "mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>(
        "mavros/set_mode");

    ros::Rate rate(20.0); // 20 Hz

    // Wait for connection to FCU
    while (ros::ok() && !current_state.connected) {
        ros::spinOnce();
        rate.sleep();
    }

    // Generate circular path waypoints
    double radius = 20.0;
    double altitude = 3.0;
    int num_waypoints = 100;
    double center_x = 0.0;
    double center_y = 0.0;

    std::vector<geometry_msgs::Point> waypoints;
    for (int i = 0; i <= num_waypoints; ++i) {
        double angle = 2 * M_PI * i / num_waypoints;
        double x = center_x + radius * cos(angle);
        double y = center_y + radius * sin(angle);
        double z = altitude;
        waypoints.push_back(makePoint(x, y, z));
    }

    // Initial setpoint
    geometry_msgs::PoseStamped pose;
    pose.pose.position = waypoints.front();

    // Send few initial setpoints before switching to OFFBOARD
    for (int i = 100; ros::ok() && i > 0; --i) {
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    // Set mode and arm vehicle
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();

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

        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }

    // Execute circular path
    double total_duration = 60.0; // seconds
    int total_steps = total_duration * 20; // total steps at 20 Hz
    int steps_per_segment = total_steps / (waypoints.size() - 1);

    for (size_t i = 0; i < waypoints.size() - 1 && ros::ok(); ++i) {
        geometry_msgs::Point start = waypoints[i];
        geometry_msgs::Point end = waypoints[i + 1];

        for (int j = 0; j < steps_per_segment && ros::ok(); ++j) {
            double t = static_cast<double>(j) / steps_per_segment;
            pose.pose.position = interpolate(start, end, t);
            local_pos_pub.publish(pose);
            ros::spinOnce();
            rate.sleep();
        }
    }

    ROS_INFO("Circular path completed.");

    return 0;
}
