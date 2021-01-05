#include <ros/ros.h>

#include <decklink_ros/ros_image_pub_thread.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "decklink_ros_node");
    ros::NodeHandle nh;

    RosImagePubThread img_pub_thread(nh);
    if (!img_pub_thread.init()) {
        ROS_ERROR("decklink_ros_node: Failed to initialize RosImagePubThread.");
        std::exit(-1);
    }

    ros::spin();

    return 0;
}
