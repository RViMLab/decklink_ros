#include <decklink_ros/ros_image_pub_thread.h>



RosImagePubThread::RosImagePubThread(ros::NodeHandle& nh) 
  : camera_name_("decklink"),
    frame_width_(0),
    frame_height_(0),
    frame_bytes_per_pixel_(0),
    frame_byte_size_(0),
    nh_(nh) {   };

RosImagePubThread::~RosImagePubThread() {
    delete decklink_capture_;
    delete publish_thread_;
    delete image_transport_;
    delete camera_info_manager_;
};

bool RosImagePubThread::init() {
    image_transport_ = new image_transport::ImageTransport(nh_);
    camera_info_manager_ = new camera_info_manager::CameraInfoManager(nh_);
    camera_info_manager_->setCameraName(camera_name_);
    image_publisher_ = image_transport_->advertiseCamera("image_raw", 1);
    publish_thread_ = new std::thread(std::bind(&RosImagePubThread::publishImages_, this));
    ros::NodeHandle privateNodeHandle("~");
    if (privateNodeHandle.hasParam("frame_id")) {
        privateNodeHandle.getParam("frame_id", frame_id_);
    }

    decklink_capture_ = new DeckLinkCapture();
    if (!decklink_capture_->init()) {
        ROS_ERROR("Failed to init DeckLinkCapture");
        return false;
    }

    decklink_capture_->setCallback(std::bind(
        &RosImagePubThread::newFrameCallback, 
        this, 
        std::placeholders::_1, 
        std::placeholders::_2, 
        std::placeholders::_3, 
        std::placeholders::_4, 
        std::placeholders::_5
        )
    );

    return true;
};

void RosImagePubThread::newFrameCallback(long unsigned frame_count, void* raw_frame, unsigned raw_frame_width, unsigned raw_frame_height, unsigned bytes_per_pixel) {
    if (frame_width_ == 0) {
        frame_width_ = raw_frame_width;
        frame_height_ = raw_frame_height;
        frame_bytes_per_pixel_ = bytes_per_pixel;
        frame_byte_size_ = frame_width_ * frame_height_ * frame_bytes_per_pixel_;
    }

    void* raw_frame_copy = malloc(frame_byte_size_);
    if (!raw_frame_copy) {
        ROS_ERROR("frame %lu: could not malloc()\n", frame_count);
        return;
    }
    memcpy(raw_frame_copy, raw_frame, frame_byte_size_);

    std::lock_guard<std::mutex> image_queue_lock(image_queue_mutex_);
    image_queue_.push(raw_frame_copy);
    frame_count_ = frame_count;
    if (image_queue_.size() > 10) {
        ROS_WARN("frame %lu: image_queue_.size()=%zd\n", frame_count, image_queue_.size());
    }
    image_queue_condition_.notify_one();
};

void RosImagePubThread::publishImages_() {
    for (;;) {
        void* frame_raw;
        {
            std::unique_lock<std::mutex> image_queue_lock(image_queue_mutex_);
            while (image_queue_.empty()) {
                image_queue_condition_.wait(image_queue_lock);
            }

            frame_raw = image_queue_.front();
            image_queue_.pop();
        }

        sensor_msgs::Image image;
        image.header.seq = frame_count_;
        image.header.stamp = ros::Time::now();
        image.header.frame_id = frame_id_;
        image.height = frame_height_;
        image.width = frame_width_;
        image.encoding = sensor_msgs::image_encodings::YUV422;

        image.step = frame_width_ * frame_bytes_per_pixel_;
        image.data = std::vector<uint8_t>((uint8_t*)frame_raw, ((uint8_t*)frame_raw) + frame_byte_size_);
        sensor_msgs::CameraInfo cameraInfo = camera_info_manager_->getCameraInfo();
        cameraInfo.header.stamp = image.header.stamp;
        cameraInfo.header.frame_id = image.header.frame_id;
        image_publisher_.publish(image, cameraInfo);

        free(frame_raw);
    }
};

