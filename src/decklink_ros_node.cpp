#include <ros/ros.h>

#include <decklink_ros/ros_image_pub_thread.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "decklink_ros_node");
    ros::NodeHandle nh;

    std::string camera_name, calibration_url;

    nh.getParam("camera_name", camera_name);
    nh.getParam("calibration_url", calibration_url);

    RosImagePubThread img_pub_thread(nh, camera_name, calibration_url);
    if (!img_pub_thread.init()) {
        ROS_ERROR("decklink_ros_node: Failed to initialize RosImagePubThread.");
        std::exit(-1);
    }

    ros::spin();

    return 0;
}
