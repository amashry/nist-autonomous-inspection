#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/GlobalPositionTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <std_msgs/String.h>
#include "utils.hpp"
#include "bucket_configuration.hpp"

/// <global variables>
// these variables contain the incoming pose data of the drone and the april tag (must be global so they can be used with callbacks)
geometry_msgs::PoseStamped geo_msg_pose_stamped_apriltag;
geometry_msgs::PoseStamped geo_msg_pose_stamped_drone;
bool geo_msg_pose_stamped_drone_data_in = 0;
bool geo_msg_pose_stamped_apriltag_data_in = 0;
const double PUBLISHING_RATE_HZ = 20.0; // in units of Hz
// Declare a global variable to keep track of the tag_id
std::string current_tag_id = "-1";


// the "1" indicates "1 second". This global variable defines the maximum number of iterations of the while-loop
// until we will no longer accept that we are currently seeing the apriltag.
const int MAX_NUMBER_OF_ITERATIONS_SINCE_LAST_SAW_APRILTAG = (int)(PUBLISHING_RATE_HZ * 0.25);

// the "8" indicates "8 seconds". This global variable defines the maximum number of iterations of the while-loop
// until we should proceed to moving to the next bucket.
const int MAX_NUMBER_OF_ITERATIONS_LOOKING_AT_BUCKET_I = (int)PUBLISHING_RATE_HZ * 5;

// tolerances for knowing whether we have reached the desired configuration
const double YAW_TOLERANCE_RAD = 3.14 / 180.0 * 15; // roughly 15 degrees, but represented in units of radians
const double POSITION_COMPONENT_TOLERANCE_M = 0.10; // 10 centimeters, but represented in units of meters

// the maximum yaw rate we ever want to command in units of rad/s
const double MAX_YAW_RATE_RAD = 3.14 / 180.0 * 25.0; // roughly 25 degrees per second, but represented in units of radians per second 
// </global variables>


/**
 * This function is a callback function to be called whenever THIS node receives a geometry_msgs::PoseStamped object 
 * published to the "/tag_detections/tagpose" rostopic.
 *
 * This geometry_msgs::PoseStamped object contains the pose of the apriltag in the camera frame.
 * 
 * inputs:
 * - [const geometry_msgs::PoseStamped::ConstPtr&] the psoe of the apriltag in the camera frame
 * outputs:
 * - NONE
 *
 * Note: this function doesn't return anything. Instead it updates some global variables. It stores the pose of the apriltag
 * in the camera frame in a global variable called "geo_msg_pose_stamped_apriltag". It also turns the global variable
 * "geo_msg_pose_stamped_apriltag_data_in" to "1" to indicate that we have received this data. The 
 * "geo_msg_pose_stamped_apriltag_data_in" variable could get changed back to "0" by the program elsewhere.
 */
void callback_pose_camera_apriltag(const geometry_msgs::PoseStamped::ConstPtr& geo_msgs_pose_stamped_apriltag_ptr){
   geo_msg_pose_stamped_apriltag = *geo_msgs_pose_stamped_apriltag_ptr;
   geo_msg_pose_stamped_apriltag_data_in = 1;  // set to true, since we must have received the data to be in this function

}

/**
 * This function is a callback function to be called whenever THIS node receives a geometry_msgs::PoseStamped object 
 * published to the "/mavros/local_position/pose" rostopic.
 *
 * This geometry_msgs::PoseStamped object contains the pose of the drone (aka the body) in the inertial frame (aka map frame).
 * 
 * inputs:
 * - [const geometry_msgs::PoseStamped::ConstPtr&] the psoe of the body in the inertial frame
 * outputs:
 * - NONE
 *
 * Note: this function doesn't return anything. Instead it updates some global variables. It stores the pose of the body
 * in the inertial frame in a global variable called "geo_msg_pose_stamped_drone". It also turns the global variable
 * "geo_msg_pose_stamped_drone_data_in" to "1" to indicate that we have received this data. The 
 * "geo_msg_pose_stamped_drone_data_in" variable could get changed back to "0" by the program elsewhere.
 */
void callback_pose_inertial_body(const geometry_msgs::PoseStamped::ConstPtr& geo_msgs_pose_stamped_drone_ptr) {
    geo_msg_pose_stamped_drone = *geo_msgs_pose_stamped_drone_ptr;
    geo_msg_pose_stamped_drone_data_in = 1;  // set to true, since we must have received the data to be in this function
}

void callback_taginfo(const std_msgs::String::ConstPtr& msg){
   current_tag_id = msg->data.c_str(); // get the tag_id as string
}


int main(int argc, char **argv) {
    // boilerplate code for node
    ros::init(argc, argv, "offboard_ctrls1"); // name the ROS node
    ros::NodeHandle nh;
    
    // declare the publishers and subscribers (pretty much more boilerplate)
    ros::Publisher pub_mavros_setpoint_raw_local = nh.advertise<mavros_msgs::PositionTarget>("/mavros/setpoint_raw/local", 10); // publish the desired position and yawrate to this topic for the autopilot
    ros::Publisher pub_pose_inertial_body_desired = nh.advertise<geometry_msgs::PoseStamped>("/pose_inertial_body_desired", 10); // publish the pose in the inertial frame where we will command the drone to go
    ros::Publisher pub_pose_inertial_apriltag = nh.advertise<geometry_msgs::PoseStamped>("/pose_inertial_apriltag", 10); // publish the pose in the inertial frame of the apriltag
    ros::Subscriber sub_pose_camera_apriltag = nh.subscribe<geometry_msgs::PoseStamped>("/tag_detections/tagpose", 1, callback_pose_camera_apriltag); // subscribe to the pose of the apriltag in the camera frame
    ros::Subscriber sub_pose_inertial_body = nh.subscribe <geometry_msgs::PoseStamped> ("/mavros/local_position/pose", 10, callback_pose_inertial_body); // subscribe to the pose of the drone in the inertial frame
    ros::Subscriber sub_taginfo = nh.subscribe<std_msgs::String>("/tag_detections/taginfo", 10, callback_taginfo); // subscribe to the taginfo topic to save tag_id 

    // Initial bucket configuration
    std::string prev_tag_id = "-1";
    std::unique_ptr<BucketConfiguration> bucket_configuration;
    
    // bucket_configuration.reset(new BucketConfiguration("/root/yoctohome/nist-autonomous-inspection/config/0.json"));
    
    // // Read the bucket configuration of the .json file we are passing in, and set the current bucket to the first bucket in the list of buckets in the .json file
    // BucketConfiguration bucket_configuration("/root/yoctohome/nist-autonomous-inspection/config/WALL.json");

    // Declare relevant poses / frames as homogeneous transformations (which are represented in code as Eigen::Matrix4d)
    Eigen::Matrix4d H_body_apriltag; // This is the pose of the apriltag in the body frame
    Eigen::Matrix4d H_inertial_apriltag; // this is the pose of the apriltag in the inertial frame
    Eigen::Matrix4d H_inertial_offset; // this is the pose of the desired position of the drone in the inertial frame

    // Set the publishing rate of THIS node... the setpoint publishing rate MUST be faster than 2Hz (for autopilot not to ignore mavros)
    ros::Rate rate(PUBLISHING_RATE_HZ); 

    // define the static fields of the "PositionTarget" struct
    mavros_msgs::PositionTarget desired_pose_inertial_body;
    desired_pose_inertial_body.coordinate_frame = 1; // this must be set to '1' for ineritial control
    desired_pose_inertial_body.type_mask = desired_pose_inertial_body.IGNORE_VX | desired_pose_inertial_body.IGNORE_VY | desired_pose_inertial_body.IGNORE_VZ | desired_pose_inertial_body.IGNORE_AFZ | desired_pose_inertial_body.IGNORE_AFY | desired_pose_inertial_body.IGNORE_AFX | desired_pose_inertial_body.IGNORE_YAW;

    // define some loop parameters that will be updated in the loop
    int number_of_iterations_since_last_saw_apriltag = 0; // number of iterations of the following while-loop in which we have seen the apriltag
    unsigned previous_apriltag_seq_number{0}; // equal to the most recent "seq" field of the "geometry_msgs::PoseStamped" struct containing apriltag pose information, in which we actually saw the apriltag.
    int number_of_iterations_at_bucket_i{0}; // number of iterations of the following while-loop in which we have been approximately in the desired configuration to see bucket "i"
    bool received_apriltag_and_drone_pose_recently{false}; // TRUE if we have gotten both an apriltag pose and a drone pose "recently". FALSE otherwise.
    bool have_seen_apriltag_at_least_once{false}; // TRUE if we have seen the apriltag one or more times. FALSE otherwise
    while (ros::ok()) {
        
        ros::spinOnce();

        // if the "seq" field of the geometry_msgs::PoseStamped object for the apriltag in the camera frame
        // is NOT equal to the previous "seq" field received from that topic, then we must be currently seeing the apriltag
        if (previous_apriltag_seq_number != geo_msg_pose_stamped_apriltag.header.seq) {
            // we reach this point iff we are currently seeing the apriltag
            previous_apriltag_seq_number = (unsigned)geo_msg_pose_stamped_apriltag.header.seq; // update the previous "seq" with the current "seq"
            number_of_iterations_since_last_saw_apriltag = 0; // make sure we reset this variable to 0
        } else {
            // we reach this point iff we are NOT currently seeing the apriltag
            ++number_of_iterations_since_last_saw_apriltag; // increment the counter. we will use this counter to infer how long it was since the last time we saw the apriltag
        }


        if (number_of_iterations_since_last_saw_apriltag == MAX_NUMBER_OF_ITERATIONS_SINCE_LAST_SAW_APRILTAG) {
            // if we have reached this point, that means that we have not seen the apriltag in 
            // "MAX_NUMBER_OF_ITERATIONS_SINCE_LAST_SAW_APRILTAG" number of iterations. This number is defined
            // at the top of the file to be [ARBITRARY NUMBER OF SECONDS] * [UPDATE FREQUENCY]. So in effect, this number,
            // "MAX_NUMBER_OF_ITERATIONS_SINCE_LAST_SAW_APRILTAG", defines the number of seconds before we should start to report
            // that we have lost sight of the apriltag.
            geo_msg_pose_stamped_apriltag_data_in = 0; // set this number to false
        }

        
        // If a new apriltag is detected, update the bucket configuration
        if (current_tag_id != prev_tag_id) {
            prev_tag_id = current_tag_id; //update the prev_tag_id 
            std::string json_file = std::string(current_tag_id) + std::string(".json");
            std::cout << "Loading config file: " << "../../config/" + json_file + ".json" << std::endl;
            bucket_configuration.reset(new BucketConfiguration(std::string("/root/yoctohome/nist-autonomous-inspection/config/") + json_file));
            }

        if (!bucket_configuration) {
            // If there's no valid bucket configuration, skip the rest of the loop
            std::string json_file = std::string("1") + std::string(".json");
            bucket_configuration.reset(new BucketConfiguration(std::string("/root/yoctohome/nist-autonomous-inspection/config/") + json_file));
        }


        // // Check if current_tag_id has been updated
        // if (current_tag_id == "0") {
        //     // Load GROUND.json
        //     ROS_INFO_STREAM("Currently Inspecting GROUND Alignmnet...");
        //     bucket_configuration.loadConfig("/root/yoctohome/nist-autonomous-inspection/config/GROUND.json");
     

        // } else if (current_tag_id == "1") {
        //     // Load WALL.json
        //     ROS_INFO_STREAM("Currently Inspecting WALL Alignmnet...");
        //     bucket_configuration.loadConfig("/root/yoctohome/nist-autonomous-inspection/config/WALL.json");
        // }

  
        // if we have received both apriltag pose data recently AND drone pose data recenently, then
        // set "received_apriltag_and_drone_pose_recently" to TRUE. otherwise set it to FALSE
        received_apriltag_and_drone_pose_recently = geo_msg_pose_stamped_apriltag_data_in && geo_msg_pose_stamped_drone_data_in;

        // Convert the pose of the drone in the inertial frame from a geometry_msgs::PoseStamped object
        // into a homogeneous tranform (as represented by a 4x4 Eigen matrix)
        Eigen::Matrix4d H_inertial_body = geo_msg_pose_stamped_to_homogeneous_tf(geo_msg_pose_stamped_drone); // [vector in inertial frame] = H_inertial_body [vector in body frame]; also represents the position of the drone in the inertial frame

        // Convert the pose of the apriltag in the camera frame from a geometry_msgs::PoseStamped object
        // into a homogeneous tranform (as represented by a 4x4 Eigen matrix)
        Eigen::Matrix4d H_camera_apriltag = geo_msg_pose_stamped_to_homogeneous_tf(geo_msg_pose_stamped_apriltag);

        // Define a static homogeneous transformation called "H_camera_body" that exhibits zero translation converts
        // transforms 4-vectors in the body frame to 4-vectors in the camera frame.
        // [vector expressed in camera frame] = H_camera_body * [vector expressed in body frame]
        // this was generated by looking at ModelAI's documentation for how the Camera frame and body frames are defined. They are that:
        // - Camera is pointing 45 deg down looking forward. = tracking camera
        Eigen::Matrix4d H_camera_body; // formerly H_M_B
        H_camera_body << 0,-1,0,0,-0.707,0,-0.707, 0, 0.707, 0, -0.707, 0, 0, 0, 0, 1;        

        if (received_apriltag_and_drone_pose_recently) {
            // we must have received an apriltag measurement and a drone position measurement in this iteration...
            // let's save the apriltag position in the inertial frame to a global variable.
            have_seen_apriltag_at_least_once = true;

            // Determine the pose of the apriltag in the body frame
            H_body_apriltag = H_camera_body.inverse() * H_camera_apriltag;

            // Determine the pose of the april tag in the local inertial frame
            H_inertial_apriltag = H_inertial_body * H_body_apriltag;

            // Print the pose of the apriltag in the body-frame
            // ROS_INFO_STREAM("Currently seeing apriltag"); // print out position?
        } else if (!have_seen_apriltag_at_least_once) {
            // if we reached this point, that means we have never seen the april tag!
            ROS_INFO_STREAM("Don't know where apriltag is");
        } else {
            // if we reached this point, that means, we are not seeing the apriltag right now AND we have seen the 
            // apriltag in the past.
            

            // Print the pose of the apriltag in the body-frame
            // ROS_INFO_STREAM("Last known position of apriltag:"); // print out position?
        }

        // Determine the desired pose of the drone so that it has the desired offset between itself and the apriltag
        Eigen::Matrix4d H_offset_apriltag = bucket_configuration->get_current_bucket_offset();
        H_inertial_offset = H_inertial_apriltag * H_offset_apriltag.inverse();

        ROS_INFO_STREAM("Currently on bucket " << bucket_configuration->get_current_bucket_name());

        // Publish the pose of the apriltag in the inertial ("map") frame to "/tag_detections/tagpose_inertial"
        geometry_msgs::PoseStamped apriltag_inertial_pub_data = homogeneous_tf_to_geo_msg_pose_stamped(H_inertial_apriltag);
        apriltag_inertial_pub_data.header.frame_id = "map";
        apriltag_inertial_pub_data.header.stamp = geo_msg_pose_stamped_drone.header.stamp;
        pub_pose_inertial_apriltag.publish(apriltag_inertial_pub_data);

        // Publish the desired pose of the drone in the inertial ("map") frame to "/desired_position"
        geometry_msgs::PoseStamped desired_inertial_pub_data = homogeneous_tf_to_geo_msg_pose_stamped(H_inertial_offset);
        desired_inertial_pub_data.header.frame_id = "map";
        desired_inertial_pub_data.header.stamp = geo_msg_pose_stamped_drone.header.stamp;
        pub_pose_inertial_body_desired.publish(desired_inertial_pub_data);

        // publish the desired pose and yawrate to /mavros/setpoint_raw/local
        desired_pose_inertial_body.header.stamp = ros::Time::now();
        desired_pose_inertial_body.position.x = H_inertial_offset(0,3);
        desired_pose_inertial_body.position.y = H_inertial_offset(1,3);
        desired_pose_inertial_body.position.z = H_inertial_offset(2,3);
        desired_pose_inertial_body.yaw_rate = (float)determine_yaw_rate(H_inertial_body, H_inertial_offset, YAW_TOLERANCE_RAD, MAX_YAW_RATE_RAD);
        pub_mavros_setpoint_raw_local.publish(desired_pose_inertial_body);

        if (std::abs(desired_pose_inertial_body.yaw_rate) < 0.000001 ) {
            ROS_INFO_STREAM("STOP YAWING");
        } else if (desired_pose_inertial_body.yaw_rate < 0) {
            ROS_INFO_STREAM("YAW CLOCKWISE");
        } else {
            ROS_INFO_STREAM("YAW COUNTERCLOCKWISE");
        }

        // increment a counter for every iteration of this while-loop in which the drone is approximately at the desired setpoint.
        if (drone_is_approximately_at_offset(H_inertial_body, H_inertial_offset, YAW_TOLERANCE_RAD, POSITION_COMPONENT_TOLERANCE_M)) {
            number_of_iterations_at_bucket_i++;
        }

        // if the counter accounting for the number of iterations of this while-loop in which the drone has been approximately
        // at the desired setpoint is equal to the maximum number of iterations, then that means we have gotten a pretty good
        // video of the bucket, so move on.
        if (number_of_iterations_at_bucket_i == MAX_NUMBER_OF_ITERATIONS_LOOKING_AT_BUCKET_I) {
            number_of_iterations_at_bucket_i = 0; // reset the counter
            bucket_configuration->increment_bucket_index(); // go to the next bucket.
        }

        // ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
