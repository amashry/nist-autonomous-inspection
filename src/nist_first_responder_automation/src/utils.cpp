#include "utils.hpp"


/**
 * represents the homogeneous transform as a string for readibility
*/
std::string homogeneous_tf_to_string(const Eigen::Matrix4d& H) {
    std::string ret("");
    ret += "[";
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            if (j == 3 && i == 3) {
                ret += std::to_string(H(i,j)) + "]\n";
            } else {
                ret += std::to_string(H(i,j)) + ", ";
            }
            
        }
        ret += "\n"; //idk
    }
    return ret;
}


/**
 * converts a homogeneous transformation (represented by a 4x4 matrix in Eigen) into a geometry_msgs::PoseStamped
 * object. 
 * inputs: 
 * - a homogeneous transformation
 * outputs:
 * - a geometry_msgs::PoseStamped object representing the same pose as was represented in the homogeneous transformation
 * 
 * Example:
 * ```c++
 * Eigen::Matrix4d H_inertial_body;
 * ...
 * geometry_msgs::PoseStamped my_pose_stamped = homogeneous_tf_to_geo_msg_pose_stamped(H_inertial_body);
 * ...
 * ```
 * so in this example the pose of the "body" (i.e. the drone) is represented in the inertial frame is encapsulated in a
 * 4x4 Matrix in Eigen. This is transformed into a geometry_msgs:PoseStamped object representing the pose of the drone
 * again in the inertial frame. 
 * 
 * Notes:
 * The homogeneous transform holds orientation information in top left 3x3 matrix in the form of a rotationa matrix. 
 * The geometry_msgs::PoseStamped object holds orientation information in a quaternion
 * The homogeneous transform holds position information in the top right 3x1 vector. 
 * The geometry_msgs::PoseStamped object holds orientation information in the same 3x1 vector.
 */
geometry_msgs::PoseStamped homogeneous_tf_to_geo_msg_pose_stamped(const Eigen::Matrix4d& homogeneous_tf) {
    // declare return object
    geometry_msgs::PoseStamped ret;

    // convert the top 3x3 rotation matrix into quaternion
    Eigen::Quaterniond quat(homogeneous_tf.topLeftCorner<3, 3>());

    // populate entries of PoseStamped object corresponding to orientation with quaternion data
    ret.pose.orientation.w = quat.w();
    ret.pose.orientation.x = quat.x();
    ret.pose.orientation.y = quat.y();
    ret.pose.orientation.z = quat.z();

    // populate entries of the PoseStamped object corresponding to position with homogeneous tf's positional data
    ret.pose.position.x = homogeneous_tf(0,3);
    ret.pose.position.y = homogeneous_tf(1,3);
    ret.pose.position.z = homogeneous_tf(2,3);

    return ret;
}

/**
 * converts a geometry_msgs::PoseStamped into a homogeneous tranformation
 */
Eigen::Matrix4d geo_msg_pose_stamped_to_homogeneous_tf(geometry_msgs::PoseStamped& msg) {
    Eigen::Matrix4d homogeneous_tf;

    // Extract the orientation of the drone in the local inertial frame. Express this orientation as a rotation matrix
    Eigen::Quaterniond quat(msg.pose.orientation.w, msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z);
    Eigen::Matrix3d rotation_matrix = quat.toRotationMatrix();

    // Populate a homogeneous tranformtation representing the pose of the drone in the local inertial frame using the 
    // data we just extracted!
    homogeneous_tf.block(0,0,4,4) = Eigen::Matrix4d::Constant(4,4, 0.0);
    homogeneous_tf.block(0,0,3,3) = rotation_matrix;
    homogeneous_tf(3,3) = 1.0;
    homogeneous_tf(0,3) = msg.pose.position.x;
    homogeneous_tf(1,3) = msg.pose.position.y;
    homogeneous_tf(2,3) = msg.pose.position.z;

    return homogeneous_tf;
}

/**
 * returns true if the drone's position and yaw are equal (within some tolerance) to the desired position and yaw
*/
bool drone_is_approximately_at_offset(const Eigen::Matrix4d& H_inertial_body, const Eigen::Matrix4d& H_inertial_offset, const double yaw_tolerance, const double position_component_tolerance) {
    // compute the yaw of the drone in the inertial frame
    Eigen::Vector3d euler_inertial_body = H_inertial_body.topLeftCorner<3, 3>().eulerAngles(2, 1, 0); // 3-2-1 euler angles, where "2" is a rot about z-axis, 0 is x-axis, and 1 is y-axis
    double yaw_inertial_body = euler_inertial_body[0];

    // compute the yaw of the offset in the inertial frame
    Eigen::Vector3d euler_inertial_offset = H_inertial_offset.topLeftCorner<3, 3>().eulerAngles(2, 1, 0); // 3-2-1 euler angles, where "2" is a rot about z-axis, 0 is x-axis, and 1 is y-axis
    double yaw_inertial_offset = euler_inertial_offset[0];

    // get the position of the drone in the inertial frame
    double x_inertial_body = H_inertial_body(0,3);
    double y_inertial_body = H_inertial_body(1,3);
    double z_inertial_body = H_inertial_body(2,3);

    // get the position of the offset in the inertial frame
    double x_inertial_offset = H_inertial_offset(0,3);
    double y_inertial_offset = H_inertial_offset(1,3);
    double z_inertial_offset = H_inertial_offset(2,3);

    // if the absolute value between the yaw and positions of the drone in the inertial frame and the offset in the inertial 
    // frame are all less than some tolerance, then that means the drone is approximately at the reference point, so return true
    bool x_within_tolerance = std::abs(x_inertial_body - x_inertial_offset) < position_component_tolerance;
    bool y_within_tolerance = std::abs(y_inertial_body - y_inertial_offset) < position_component_tolerance;
    bool z_within_tolerance = std::abs(z_inertial_body - z_inertial_offset) < position_component_tolerance;
    bool yaw_within_tolerance = std::abs(yaw_inertial_body - yaw_inertial_offset) < yaw_tolerance;

    // only return true if all of the abolute values are within the tolerances
    return x_within_tolerance && y_within_tolerance && z_within_tolerance && yaw_within_tolerance;
}

/**
 * determines the yawrate to publish to /mavros/setpoint_raw/local to guarantee that we face the right direction
*/
double determine_yaw_rate(const Eigen::Matrix4d& H_inertial_body, const Eigen::Matrix4d H_inertial_offset, const double yaw_tolerance, const double max_yaw_rate) {
    // determine the yaw (in range (0, 2pi)) of the drone w.r.t. the inertial frame
    double yaw_inertial_body = homogeneous_tf_to_yaw(H_inertial_body, true);
    // ROS_INFO_STREAM("yaw inertial body " << std::to_string(yaw_inertial_body));

    // determine the yaw (in range (0, 2pi)) of the offset w.r.t. the inertial frame
    double yaw_inertial_offset = homogeneous_tf_to_yaw(H_inertial_offset, false);
    // ROS_INFO_STREAM("yaw inertial offset " << std::to_string(yaw_inertial_offset));

    // take difference from target to source
    double smallest_angle_difference = yaw_inertial_offset - yaw_inertial_body;
    // ROS_INFO_STREAM("smallest angle difference " << std::to_string(smallest_angle_difference));

    if (smallest_angle_difference > 3.14) {
        smallest_angle_difference -= 2 * 3.14;
    } else if (smallest_angle_difference < -3.14) {
        smallest_angle_difference += 2 * 3.14;
    }

    // determine the yawrate to publish given the smallest angle between actual yaw and desired yaw
    if (std::abs(smallest_angle_difference) < yaw_tolerance / 2) {
        // if the absolute difference between the smallest angle between the drone and the desired is less than 1/2 the tolerance, then just stop yawing
        return 0.0;
    } else if (smallest_angle_difference < 0) {
        // if the smallest angle difference is negative, then command a negative yaw rate
        return -max_yaw_rate;
    }

    // if we reached this point, then the smallest angle difference must be > 1/2 the tolerance AND
    // the smallest angle must be positive, so command a positive yawrate!
    return max_yaw_rate;
}

/**
 * this function returns the yaw of the given homogenous transform in the range of (0, 2pi)
*/
double homogeneous_tf_to_yaw(const Eigen::Matrix4d& homogeneous_tf, bool verbose) {

    // determine the 3-2-1 Euler angles (these go from (0, pi) and (0, pi))
    Eigen::Vector3d desired_euler_angles_3_2_1 = homogeneous_tf.topLeftCorner<3, 3>().eulerAngles(2, 1, 0); // 3-2-1 euler angles, where "2" is a rot about z-axis, 0 is x-axis, and 1 is y-axis
    // if (verbose) {
    //     ROS_INFO_STREAM("desired_euler_angles_3_2_1 " << std::to_string(desired_euler_angles_3_2_1[0]));
    // }

    // determine the 2-3-1 Euler angles (these go from (0, pi) and (0, pi))
    Eigen::Vector3d desired_euler_angles_2_3_1 = homogeneous_tf.topLeftCorner<3, 3>().eulerAngles(1,2,0); // 3-2-1 euler angles, where "2" is a rot about z-axis, 0 is x-axis, and 1 is y-axis

    // if 2-3-1 yaw is negative, then we can add pi to the 3-2-1 yaw to get the yaw from (0, 2pi)
    if (desired_euler_angles_2_3_1[1] < 0) {
        desired_euler_angles_3_2_1[0] += 3.1415;
    }

    return desired_euler_angles_3_2_1[0]; // i believe technically that we need to subtract pi from this, but doesn't really matter?
}