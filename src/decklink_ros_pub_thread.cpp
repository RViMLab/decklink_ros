#include "decklink_ros/decklink_ros_pub_thread.hpp"

DeckLinkRosPubThread::DeckLinkRosPubThread() : Node("decklink_ros_pub_thread") {
  this->declare_parameter("cname", "decklink");
  this->declare_parameter("url", "");
  this->declare_parameter("frame_id", "");

  this->get_parameter("cname", cname_);
  this->get_parameter("url", url_);
  this->get_parameter("frame_id", frame_id_);

  frame_width_ = 0;
  frame_height_ = 0;
  frame_bytes_per_pixel_ = 0;
  frame_byte_size_ = 0;
}

bool DeckLinkRosPubThread::init() {
  image_transport_ = std::make_unique<image_transport::ImageTransport>(this->shared_from_this());
  camera_info_manager_ = std::make_unique<camera_info_manager::CameraInfoManager>(
      this->shared_from_this().get(), cname_, url_);
  image_publisher_ = image_transport_->advertiseCamera("image_raw", 1);
  publish_thread_ =
      std::make_unique<std::thread>(std::bind(&DeckLinkRosPubThread::publishImages_, this));

  decklink_capture_ = std::make_unique<DeckLinkCapture>();
  if (!decklink_capture_->init()) {
    RCLCPP_ERROR(this->get_logger(), "Failed to init DeckLinkCapture");
    return false;
  }

  decklink_capture_->setCallback(std::bind(
      &DeckLinkRosPubThread::newFrameCallback, this, std::placeholders::_1, std::placeholders::_2,
      std::placeholders::_3, std::placeholders::_4, std::placeholders::_5));

  return true;
}

void DeckLinkRosPubThread::newFrameCallback(long unsigned frame_count, void *raw_frame,
                                            unsigned raw_frame_width, unsigned raw_frame_height,
                                            unsigned bytes_per_pixel) {
  if (frame_width_ == 0) {
    frame_width_ = raw_frame_width;
    frame_height_ = raw_frame_height;
    frame_bytes_per_pixel_ = bytes_per_pixel;
    frame_byte_size_ = frame_width_ * frame_height_ * frame_bytes_per_pixel_;
  }

  void *raw_frame_copy = malloc(frame_byte_size_);
  if (!raw_frame_copy) {
    RCLCPP_ERROR(this->get_logger(), "Frame %lu: could not malloc()\n", frame_count);
    return;
  }
  memcpy(raw_frame_copy, raw_frame, frame_byte_size_);

  std::lock_guard<std::mutex> image_queue_lock(image_queue_mutex_);
  image_queue_.push(raw_frame_copy);
  frame_count_ = frame_count;
  if (image_queue_.size() > 10) {
    RCLCPP_WARN(this->get_logger(), "Frame %lu: image_queue_.size()=%zd\n", frame_count,
                image_queue_.size());
  }
  image_queue_condition_.notify_one();
}

void DeckLinkRosPubThread::publishImages_() {
  for (;;) {
    void *frame_raw;
    {
      std::unique_lock<std::mutex> image_queue_lock(image_queue_mutex_);
      while (image_queue_.empty()) {
        image_queue_condition_.wait(image_queue_lock);
      }

      frame_raw = image_queue_.front();
      image_queue_.pop();
    }

    // convert to bgr
    cv::Mat img_cv(frame_height_, frame_width_, CV_8UC2, frame_raw);
    cv::cvtColor(img_cv, img_cv, cv::COLOR_YUV2BGR_UYVY);

    // create bridge
    std_msgs::msg::Header header;
    header.stamp = this->now();
    header.frame_id = frame_id_;
    cv_bridge::CvImage img_bridge(header, sensor_msgs::image_encodings::BGR8, img_cv);

    // create image message
    sensor_msgs::msg::Image img_msg;
    img_bridge.toImageMsg(img_msg);

    auto camera_info = camera_info_manager_->getCameraInfo();
    camera_info.header = header;
    image_publisher_.publish(img_msg, camera_info);

    free(frame_raw);
  }
}
