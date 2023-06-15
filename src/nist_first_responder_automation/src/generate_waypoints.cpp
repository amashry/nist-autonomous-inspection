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

geometry_msgs::PoseStamped geo_msg_pose_stamped_apriltag;
geometry_msgs::PoseStamped geo_msg_pose_stamped_drone;
unsigned previous_at_in_seq{0}; // rename later...


bool geo_msg_pose_stamped_drone_data_in = 0;
bool geo_msg_pose_stamped_apriltag_data_in = 0;
bool all_in = 0;

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

void at_cb(const geometry_msgs::PoseStamped::ConstPtr& geo_msgs_pose_stamped_apriltag_ptr){
   geo_msg_pose_stamped_apriltag = *geo_msgs_pose_stamped_apriltag_ptr;
   geo_msg_pose_stamped_apriltag_data_in = 1;  // set to true, since we must have received the data to be in this function

}

void geo_msg_pose_stamped_drone_callback(const geometry_msgs::PoseStamped::ConstPtr& geo_msgs_pose_stamped_drone_ptr) {
    geo_msg_pose_stamped_drone = *geo_msgs_pose_stamped_drone_ptr;
    geo_msg_pose_stamped_drone_data_in = 1;  // set to true, since we must have received the data to be in this function
}


int main(int argc, char **argv) {
    // assign these to zero why?
    geo_msg_pose_stamped_apriltag.pose.position.x = 0;
    geo_msg_pose_stamped_apriltag.pose.position.y = 0;
    geo_msg_pose_stamped_apriltag.pose.position.z = 0;


    // boilerplate code for node
    ros::init(argc, argv, "generate_waypoints_node"); // name the ROS node
    ros::NodeHandle nh;
    
    // .///
    ros::Publisher local_pos_pub_mavros = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 5);
    ros::Subscriber apriltag_sub = nh.subscribe<geometry_msgs::PoseStamped>("/tag_detections/tagpose", 1, at_cb);
    ros::Subscriber local_info_sub = nh.subscribe <geometry_msgs::PoseStamped> ("/mavros/local_position/pose", 10, geo_msg_pose_stamped_drone_callback);
    ros::Publisher target_body_pub = nh.advertise<geometry_msgs::PoseStamped>("/tag_detections/tagpose_body", 10);
    ros::Publisher target_lpp_pub = nh.advertise<geometry_msgs::PoseStamped>("/tag_detections/tagpose_inertial", 10);
    ros::Publisher target_body_stab_pub = nh.advertise<geometry_msgs::PoseStamped>("/tag_detections/tagpose_body_stabilized", 10);

    //the setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    ros::Time last_request = ros::Time::now();

    int count = 0;
    int last_button_val = 0;
    float phase = 0;
    double sinusoid_val;
    
    while(ros::ok()) {

        if (previous_at_in_seq != geo_msg_pose_stamped_apriltag.header.seq) {
            previous_at_in_seq = (unsigned)geo_msg_pose_stamped_apriltag.header.seq;
            count = 0;
        } else {
            ++count;
        }

        if (count == 20) {
            // recall rate is 20 Hz, therefore, 20 cycles is equivalent to 20 / 20 = 1 seconds
            geo_msg_pose_stamped_apriltag_data_in = 0;
        }
  
        // document me...
        if(geo_msg_pose_stamped_apriltag_data_in && geo_msg_pose_stamped_drone_data_in){
            all_in = 1;
        } else {
            all_in = 0;
        }

        // Extract the position of the drone in the local inertial frame. 
        // Recall: that when starting the vehicle indoors, the the y-axis of the local inertial frame is defined as the direction
        // the drone is facing upon starting up!
        // Recall: that the body frame of the drone is an Forward-left-up (FLU) frame:
        // - x-axis points out the front of the drone
        // - y-axis points out the 'left' side of the drone
        // z- -axis points out the top of the drone
        double x_drone_ineritial = geo_msg_pose_stamped_drone.pose.position.x;
        double y_drone_inertial = geo_msg_pose_stamped_drone.pose.position.y;
        double z_drone_inertial = geo_msg_pose_stamped_drone.pose.position.z;

        // Extract the orientation of the drone in the local inertial frame. Express this orientation as a rotation matrix
        Eigen::Quaterniond quaternion_drone_inertial(geo_msg_pose_stamped_drone.pose.orientation.w, geo_msg_pose_stamped_drone.pose.orientation.x, geo_msg_pose_stamped_drone.pose.orientation.y, geo_msg_pose_stamped_drone.pose.orientation.z);
        Eigen::Matrix3d R_drone_inertial = quaternion_drone_inertial.toRotationMatrix();

        // Populate a homogeneous tranformtation representing the pose of the drone in the local inertial frame using the 
        // data we just extracted!
        Eigen::Matrix4d H_drone_inerital;
        H_drone_inerital.block(0,0,4,4) = Eigen::Matrix4d::Constant(4,4, 0.0);
        H_drone_inerital.block(0,0,3,3) = R_drone_inertial;
        H_drone_inerital(3,3) = 1.0;
        H_drone_inerital(0,3) = x_drone_ineritial;
        H_drone_inerital(1,3) = y_drone_inertial;
        H_drone_inerital(2,3) = z_drone_inertial;

        // Extract the position of the apriltag in the camera frame. Recall:
        // - Z is coincident with optical axis; 
        // - Y is "down" in camera frame; 
        // - X is "to-the-right" when looking from vehicle out
        double x_apriltag_camera = geo_msg_pose_stamped_apriltag.pose.position.x;
        double y_apriltag_camera = geo_msg_pose_stamped_apriltag.pose.position.y;
        double z_apriltag_camera = geo_msg_pose_stamped_apriltag.pose.position.z;

        // Extract the orientation of the april tag frame w.r.t. the camera frame. Express this orientation as a rotation matrix
        Eigen::Quaterniond quat_apriltag_camera(geo_msg_pose_stamped_apriltag.pose.orientation.w, geo_msg_pose_stamped_apriltag.pose.orientation.x, geo_msg_pose_stamped_apriltag.pose.orientation.y, geo_msg_pose_stamped_apriltag.pose.orientation.z); // recall that Eigen's convention for quaternions is to have the 'w' scalar first
        Eigen::Matrix3d R_apriltag_camera = quat_apriltag_camera.toRotationMatrix();
        
        // Create a homogeneous transformation (representing the pose of the Apriltag in the Camera frame) with the position 
        // and orientation information we just extracted
        Eigen::Matrix4d H_apriltag_camera;
        H_apriltag_camera.block(0,0,4,4) = Eigen::Matrix4d::Constant(4,4, 0.0);
        H_apriltag_camera.block(0,0,3,3) = R_apriltag_camera;
        H_apriltag_camera(3,3) = 1.0;
        H_apriltag_camera(0,3) = x_apriltag_camera;
        H_apriltag_camera(1,3) = y_apriltag_camera;
        H_apriltag_camera(2,3) = z_apriltag_camera;


        // Define a static homogeneous transformation called "H_camera_body" that exhibits zero translation converts
        // transforms 4-vectors in the body frame to 4-vectors in the camera frame.
        // [vector expressed in camera frame] = H_camera_body * [vector expressed in body frame]
        // this was generated by looking at ModelAI's documentation for how the Camera frame and body frames are defined. They are that:
        // - Camera is pointing 45 deg down looking forward. = tracking camera
        Eigen::Matrix4d H_camera_body; // formerly H_M_B
        H_camera_body << 0,-1,0,0,-0.707,0,-0.707, 0, 0.707, 0, -0.707, 0, 0, 0, 0, 1;

        // Determine the pose of the apriltag in the body frame
        Eigen::Matrix4d H_apriltag_body;
        H_apriltag_body = H_camera_body.inverse() * H_apriltag_camera;

        // Print the pose of the apriltag in the body-frame
        ROS_INFO_STREAM("April tag pose in body-frame:\n" << homogeneous_tf_to_string(H_apriltag_body));

        
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
