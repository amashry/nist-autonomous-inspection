#include "utils.hpp"


/**
 * returns true if the drone's position is equal (within predefined tolerance) to the desired waypoint position 
*/
bool drone_is_approximately_at_search_waypoint(const mavros_msgs::PositionTarget waypoint, 
                                        const geometry_msgs::PoseStamped current_pose,
                                        const double position_component_tolerance) 
{
    // extract position values from PositionTarget msgs and PoseStamped msgs 
    double x_inertial_desired = waypoint.position.x;
    double y_inertial_desired = waypoint.position.y; 
    double z_inertial_desired = waypoint.position.z; 


    double x_inertial_current = current_pose.pose.position.x;
    double y_inertial_current = current_pose.pose.position.y;
    double z_inertial_current = current_pose.pose.position.z;

    // if the absolute value between the positions of the drone in the inertial frame and the desired waypoint in the inertial 
    // frame are all less than some tolerance, then that means the drone is approximately at the reference point, so return true
    bool x_within_tolerance = std::abs(x_inertial_desired - x_inertial_current) < position_component_tolerance;
    bool y_within_tolerance = std::abs(y_inertial_desired - y_inertial_current) < position_component_tolerance;
    bool z_within_tolerance = std::abs(z_inertial_desired - z_inertial_current) < position_component_tolerance;

    // only return true if all of the abolute values are within the tolerances
    // WE CAN PLAY WITH if we want to change drone search speed 
    // I suggest to OR x,y with z because it's more important to maintain x,y 
    // to follow the path, compared with puttin a hard constraint on the altitude 
    return x_within_tolerance && y_within_tolerance && z_within_tolerance;
}



std::vector<mavros_msgs::PositionTarget> waypoints;
/**
 * brief Generates waypoints for a lawnmower-style search pattern.
 * 
 * param length Length of the search area. (ALONG Y AXIS OR NORTH for ENU frame)
 * param width Width of the search area. (ALONG X AXIS OR EAST for ENU frame)
 * param altitude Fixed Altitude at which the drone should fly.
 * param interval Number of intervals between parallel lines. 
 * param search_time_sec Time to cover the search area, in seconds.
 * return void
 */

void generate_search_waypoints(double length, double width, double altitude, int interval, int search_time_sec){
    // Define the number of steps
    int num_of_steps = PUBLISHING_RATE_HZ * search_time_sec;

    // Define the area limits here
    double x0 = 0.0, y0 = 0.0, x_W = x0 + width, y_L = y0 + length;

    // Fixed altitude
    double z = altitude;

    // Step size along x axis (EAST OR TO THE RIGHT OF THE DRONE)
    double step_size = (interval+1)*width / num_of_steps;

    // Boolean to toggle direction
    bool forward = true;

    for (double y = y0; y <= y_L; y += length/interval){
        mavros_msgs::PositionTarget waypoint;
        waypoint.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        
        waypoint.type_mask = mavros_msgs::PositionTarget::IGNORE_VX + mavros_msgs::PositionTarget::IGNORE_VY +
                             mavros_msgs::PositionTarget::IGNORE_VZ + mavros_msgs::PositionTarget::IGNORE_AFX +
                             mavros_msgs::PositionTarget::IGNORE_AFY + mavros_msgs::PositionTarget::IGNORE_AFZ +
                             mavros_msgs::PositionTarget::FORCE + mavros_msgs::PositionTarget::IGNORE_YAW; 
        
        // waypoint.yaw = PI/2;
        waypoint.velocity.x = 1; 
        waypoint.velocity.y = 1;
        waypoint.position.z = z;
        waypoint.position.y = y;

        // Generate waypoints
        // if (forward){
        //     for (double x = x0; x <= x_W; x += step_size){
        //         waypoint.position.x = x;
        //         waypoints.push_back(waypoint);
        //     }
        // } else {
        //     for (double x = x_W; x >= x0; x -= step_size){
        //         waypoint.position.x = x;
        //         waypoints.push_back(waypoint);
        //     }
        // }

                // Generate waypoints
        if (forward){
            for (double x = x0; x <= x_W; x += x_W/3){
                waypoint.position.x = x;
                waypoints.push_back(waypoint);
            }
        } else {
            for (double x = x_W; x >= x0; x -= x_W/3){
                waypoint.position.x = x;
                waypoints.push_back(waypoint);
            }
        }

        
        // Switch direction
        forward = !forward;
    }
}

std::vector<mavros_msgs::PositionTarget> get_search_waypoints() {
    return waypoints;
}


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

// Function to calculate the best pose out of multiple pose estimates 
Eigen::Matrix4d bestPose(const std::vector<Eigen::Matrix4d>& poses) {
    // Step 1: Create vectors to hold translation and rotation parts separately
    std::vector<Eigen::Vector3d> translations;
    std::vector<Eigen::Quaterniond> rotations;

    for (const auto& H : poses) {
        // split the 4x4 matrix into translation and rotation
        Eigen::Vector3d translation = H.block<3, 1>(0, 3);
        Eigen::Matrix3d rotation = H.block<3, 3>(0, 0);

        translations.push_back(translation);
        rotations.push_back(Eigen::Quaterniond(rotation));
    }

    // Step 2: Use outlier rejection for translations
    translations = reject_outliers(translations);

    // Step 3: Average translations
    Eigen::Vector3d avg_translation = Eigen::Vector3d::Zero();
    for (const auto& translation : translations) {
        avg_translation += translation;
    }
    avg_translation /= translations.size();

    // Step 4: Average rotations

    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(4, 4);
    for (const auto& rotation : rotations) {
        Eigen::Vector4d q = rotation.coeffs();
        Q += q * q.transpose();  // outer product
    }

    // normalization
    Q /= rotations.size();

    Eigen::JacobiSVD<Eigen::MatrixXd> svd(Q, Eigen::ComputeThinU);
    Eigen::Vector4d avg_rotation_coeffs = svd.matrixU().col(0);
    Eigen::Quaterniond avg_rotation(avg_rotation_coeffs(3), avg_rotation_coeffs(0), avg_rotation_coeffs(1), avg_rotation_coeffs(2));
    avg_rotation.normalize();

    // Step 5: Combine average translation and rotation into a 4x4 matrix
    Eigen::Matrix4d avg_transform = Eigen::Matrix4d::Identity();
    avg_transform.block<3, 1>(0, 3) = avg_translation;
    avg_transform.block<3, 3>(0, 0) = avg_rotation.toRotationMatrix();

    return avg_transform;
}

std::vector<Eigen::Vector3d> reject_outliers(const std::vector<Eigen::Vector3d>& translations) {
    // compute the mean
    Eigen::Vector3d mean = Eigen::Vector3d::Zero();
    for (const auto& translation : translations) {
        mean += translation;
    }
    mean /= translations.size();

    // compute the standard deviation
    Eigen::Vector3d std_dev = Eigen::Vector3d::Zero();
    for (const auto& translation : translations) {
        Eigen::Vector3d diff = translation - mean;
        std_dev += diff.cwiseProduct(diff);
    }
    std_dev = (std_dev / translations.size()).cwiseSqrt();

    // reject outliers
    std::vector<Eigen::Vector3d> result;
    for (const auto& translation : translations) {
        Eigen::Vector3d z_score = (translation - mean).cwiseQuotient(std_dev);
        if ((z_score.array().abs() < 2).all()) {  // here "2" represents the threshold for the z-score
            result.push_back(translation);
        }
    }

    return result;
}
