#ifndef UTILS_HPP
#define UTILS_HPP

#include <ros/ros.h>
#include <Eigen/Dense>
#include <string>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/PositionTarget.h>
#include <cmath> // for std::pow
#include "bucket_configuration.hpp"

#define PI                    3.14159265358979323846264338327950
#define PUBLISHING_RATE_HZ    20

enum Bucket {
    Bucket1,
    Bucket1a,
    Bucket2,
    Bucket2a,
    Bucket3,
    Bucket3a,
    Bucket4,
    Bucket4a
};

// Function declarations
std::vector<Eigen::Vector3d> reject_outliers(const std::vector<Eigen::Vector3d>& translations); 
Eigen::Matrix4d bestPose(const std::vector<Eigen::Matrix4d>& poses);

void generate_search_waypoints(double length, double width, double altitude, int interval, int search_time_sec);
std::vector<mavros_msgs::PositionTarget> get_search_waypoints();
bool drone_is_approximately_at_search_waypoint(const mavros_msgs::PositionTarget waypoint, 
                                        const geometry_msgs::PoseStamped current_pose,
                                        const double position_component_tolerance);

std::string homogeneous_tf_to_string(const Eigen::Matrix4d&);
geometry_msgs::PoseStamped homogeneous_tf_to_geo_msg_pose_stamped(const Eigen::Matrix4d&);
Eigen::Matrix4d geo_msg_pose_stamped_to_homogeneous_tf(geometry_msgs::PoseStamped&);
bool drone_is_approximately_at_offset(const Eigen::Matrix4d&, const Eigen::Matrix4d&, const double, const double);
double determine_yaw_rate(const Eigen::Matrix4d&, const Eigen::Matrix4d, const double, const double);
double homogeneous_tf_to_yaw(const Eigen::Matrix4d&, bool);

#endif // UTILS_HPP