#include <rclcpp/rclcpp.hpp>

#include <decklink_ros/decklink_ros_pub_thread.hpp>


int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DeckLinkRosPubThread>();
    if (!node->init()) {
        RCLCPP_ERROR(node->get_logger(), "Failed to initialize.");
        return -1;
    }
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
