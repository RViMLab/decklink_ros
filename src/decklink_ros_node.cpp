#include <ros/ros.h>

#include <decklink_ros/decklink_capture.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "decklink_ros_node");
    ros::NodeHandle nh;

    int device_id, mode_id, pixel_format_id;
    
    nh.getParam("decklink/device_id", device_id);
    nh.getParam("decklink/mode_id", mode_id);
    nh.getParam("decklink/pixelformat_id", pixel_format_id);

    DeckLinkCapture capture();

    ros::spin();

    return 0;
}
