#pragma once

#include <memory>
#include <mutex>
#include <thread>
#include <string>
#include <exception>
#include <queue>
#include <condition_variable>

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <camera_info_manager/camera_info_manager.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <decklink_capture.hpp>


class DeckLinkRosPubThread : public rclcpp::Node {
    public:
        DeckLinkRosPubThread();
        ~DeckLinkRosPubThread() = default;

        bool init();
        void newFrameCallback(long unsigned frame_count, void* raw_frame, unsigned raw_frame_width, unsigned raw_frame_height, unsigned bytes_per_pixel);

    private:
        void publishImages_();
        
        // parameters
        std::string cname_;
        std::string url_;

        // decklink capture
        std::unique_ptr<DeckLinkCapture> decklink_capture_;

        // frame 
        unsigned frame_width_;
        unsigned frame_height_;
        unsigned frame_bytes_per_pixel_;
        unsigned frame_byte_size_;
        unsigned frame_count_; // protected by image_queue_mutex_
        std::string frame_id_;

        std::mutex image_queue_mutex_;
        std::condition_variable image_queue_condition_;
        std::queue<void*> image_queue_;
        std::unique_ptr<std::thread> publish_thread_;

        image_transport::CameraPublisher image_publisher_;
        std::unique_ptr<image_transport::ImageTransport> image_transport_;
        std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
};
