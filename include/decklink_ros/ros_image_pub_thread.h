#pragma once

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <camera_info_manager/camera_info_manager.h>
#include <mutex>
#include <thread>
#include <queue>
#include <condition_variable>

#include <decklink_capture.h>

class RosImagePubThread {
    public:
        RosImagePubThread(ros::NodeHandle& nh, const std::string& camera_name, const std::string& calibration_url);

        ~RosImagePubThread();

        bool init();
        void newFrameCallback(long unsigned frame_count, void* raw_frame, unsigned raw_frame_width, unsigned raw_frame_height, unsigned bytes_per_pixel);

    private:
        void publishImages_();

        DeckLinkCapture* decklink_capture_;

        const std::string camera_name_;
        const std::string calibration_url_;

        unsigned frame_width_;
        unsigned frame_height_;
        unsigned frame_bytes_per_pixel_;
        unsigned frame_byte_size_;
        unsigned frame_count_; // protected by image_queue_mutex_
        std::string frame_id_;

        std::mutex image_queue_mutex_;
        std::condition_variable image_queue_condition_;
        std::queue<void*> image_queue_;
        std::thread* publish_thread_;

        ros::NodeHandle& nh_;
        image_transport::CameraPublisher image_publisher_;
        image_transport::ImageTransport* image_transport_;
        camera_info_manager::CameraInfoManager* camera_info_manager_;
};
