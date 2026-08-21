#include "go2_lowlevel/camera.hpp"

namespace go2_lowlevel {

Camera::Camera(rclcpp::Node::SharedPtr node, const std::string &ns, std::vector<rclcpp::Parameter> parameters)
    : node_(node) {

    node_->declare_parameter<std::string>(
        "camera_calibration_file",
     "");

    const std::string calibration_file =
        node_->get_parameter("camera_calibration_file").as_string();

    load_camera_calibration(calibration_file);
    
    camera_sub_ = node_->create_subscription<unitree_go::msg::Go2FrontVideoData>(ns + "/frontvideostream", qos_, std::bind(&Camera::camera_callback, this, std::placeholders::_1));
    image_pub_ = node_->create_publisher<sensor_msgs::msg::Image>(ns + "/camera/image_raw", qos_);
    camera_info_pub_ = node_->create_publisher<sensor_msgs::msg::CameraInfo>(ns + "/camera/camera_info", qos_);

    codec_ = avcodec_find_decoder(AV_CODEC_ID_H264);
    codec_context_ = avcodec_alloc_context3(codec_);
    packet_ = av_packet_alloc();
    frame_ = av_frame_alloc();

    processing_thread_ = std::thread(&Camera::processing_loop, this);

    RCLCPP_INFO(node_->get_logger(), "Camera node initialized");

}

Camera::~Camera() {
    running_ = false;
    queue_cv_.notify_all();
    if (processing_thread_.joinable()) {
        processing_thread_.join();
    }
    av_frame_free(&frame_);
    av_packet_free(&packet_);
    avcodec_free_context(&codec_context_);
}

void Camera::load_camera_calibration(const std::string &path) {
    RCLCPP_INFO(node_->get_logger(), "Loading camera calibration from: %s", path.c_str());
    camera_info_msg_.header.frame_id = "camera";
    camera_info_msg_.header.stamp = node_->now();
    camera_info_msg_.height = 360;
    camera_info_msg_.width = 640;
    camera_info_msg_.distortion_model = "plumb_bob";
    camera_info_msg_.d = {0.0, 0.0, 0.0, 0.0, 0.0};
    camera_info_msg_.k = {1.0, 0.0, static_cast<double>(640) / 2.0,
                            0.0, 1.0, static_cast<double>(360) / 2.0,
                            0.0, 0.0, 1.0};
    camera_info_msg_.r = {1.0, 0.0, 0.0,
                            0.0, 1.0, 0.0,
                            0.0, 0.0, 1.0};
    camera_info_msg_.p = {1.0, 0.0, static_cast<double>(640) / 2.0, 0.0,
                            0.0, 1.0, static_cast<double>(360) / 2.0, 0.0,
                            0.0, 0.0, 1.0, 0.0};
    if (path.empty()) {
        RCLCPP_WARN(node_->get_logger(), "Camera calibration file path is empty. Using default calibration.");
        return;
    }
    try {
        const YAML::Node config = YAML::LoadFile(path);
        if (config["image_width"]) {
            camera_info_msg_.width = config["image_width"].as<int>();
        }
        if (config["image_height"]) {
            camera_info_msg_.height = config["image_height"].as<int>();
        }
        if (config["distortion_model"]) {
            camera_info_msg_.distortion_model = config["distortion_model"].as<std::string>();
        }
        if (config["camera_matrix"] && config["camera_matrix"]["data"])
        {
            const auto data = config["camera_matrix"]["data"];
            for (size_t i = 0; i < 9; ++i) {
                camera_info_msg_.k[i] = data[i].as<double>();
            }
        }
        if (config["distortion_coefficients"] && config["distortion_coefficients"]["data"])
        {
            const auto data = config["distortion_coefficients"]["data"];
            camera_info_msg_.d.resize(data.size());
            for (size_t i = 0; i < data.size(); ++i) {
                camera_info_msg_.d[i] = data[i].as<double>();
            }
        }
        if (config["rectification_matrix"] && config["rectification_matrix"]["data"])
        {
            const auto data = config["rectification_matrix"]["data"];
            for (size_t i = 0; i < 9; ++i) {
                camera_info_msg_.r[i] = data[i].as<double>();
            }
        }
        if (config["projection_matrix"] && config["projection_matrix"]["data"])
        {
            const auto data = config["projection_matrix"]["data"];
            for (size_t i = 0; i < 12; ++i) {
                camera_info_msg_.p[i] = data[i].as<double>();
            }
        }
    }
    catch (const std::exception &e) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to load camera calibration: %s. Using defaults calibration.", e.what());
    }

}

void Camera::camera_callback(const unitree_go::msg::Go2FrontVideoData::SharedPtr msg) {
    if(msg->resolution != 360 || msg->data.size() < 5) return;

    const uint8_t nal_type = msg->data[4] & 0x1F;

    {
        std::lock_guard<std::mutex> lock(queue_mutex_);
        if(nal_type != 1) {
            synced_ = true;
            frame_queue_.clear();
        }
        if(synced_ && frame_queue_.size() < MAX_QUEUE_SIZE) {
            frame_queue_.push_back(msg);
            queue_cv_.notify_one();
        }
    }

}

void Camera::processing_loop()
{
    if (avcodec_open2(codec_context_, codec_, nullptr) < 0) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to open H264 decoder");
        return;
    }

    SwsContext *sws = nullptr;

    while(running_)
    {
        unitree_go::msg::Go2FrontVideoData::SharedPtr msg;

        {
            std::unique_lock<std::mutex> lock(queue_mutex_);

            queue_cv_.wait(lock, [this]() {
                return !running_ || !frame_queue_.empty();
            });

            if (!running_) {
                break;
            }

            msg = frame_queue_.front();
            frame_queue_.pop_front();
        }

        av_packet_unref(packet_);
        packet_->data = const_cast<uint8_t*>(msg->data.data());
        packet_->size = msg->data.size();

        int ret = avcodec_send_packet(codec_context_, packet_);
        if (ret < 0) {
            RCLCPP_WARN(node_->get_logger(), "Failed to send message to decoder: %d", ret);
            continue;
        }

        ret = avcodec_receive_frame(codec_context_, frame_);
        if (ret == AVERROR(EAGAIN) || ret < 0) {
            RCLCPP_WARN(node_->get_logger(), "Decoder not ready to receive frame: %d", ret);
            continue;
        }

        if(!sws) {
            sws = sws_getContext(frame_->width, frame_->height, static_cast<AVPixelFormat>(frame_->format),
                                 frame_->width, frame_->height, AV_PIX_FMT_BGR24,
                                 SWS_BILINEAR, nullptr, nullptr, nullptr);
        }

        const int width = frame_->width;
        const int height = frame_->height;
        const int step = width * 3;
        std::vector<uint8_t> bgr(static_cast<size_t>(step * height));
        uint8_t *dest[4] = { bgr.data(), nullptr, nullptr, nullptr };
        int dest_stride[4] = { step, 0, 0, 0 };

        sws_scale(sws, frame_->data, frame_->linesize, 0, height, dest, dest_stride);

        sensor_msgs::msg::Image image_msg;
        image_msg.header.stamp = node_->now();
        image_msg.header.frame_id = "camera_link";
        image_msg.height = height;
        image_msg.width = width;
        image_msg.encoding = "bgr8";
        image_msg.is_bigendian = false;
        image_msg.step = step;
        image_msg.data = std::move(bgr);
        image_pub_->publish(image_msg);

        sensor_msgs::msg::CameraInfo camera_info_msg = camera_info_msg_;
        camera_info_msg.header.stamp = image_msg.header.stamp;
        camera_info_msg.header.frame_id = image_msg.header.frame_id;
        camera_info_msg.height = image_msg.height;
        camera_info_msg.width = image_msg.width;
        camera_info_pub_->publish(camera_info_msg);
    }

    if (sws) {
        sws_freeContext(sws);
        sws = nullptr;
    }
}

}  // namespace go2_lowlevel


int main(int argc, char **argv) 
{
  // init node
  rclcpp::init(argc, argv);

  // Create ros node
  auto node = std::make_shared<rclcpp::Node>("camera");

  // Create cpp node
  [[maybe_unused]] go2_lowlevel::Camera camera(node);

  // Create executor
  rclcpp::experimental::executors::EventsExecutor exec;
  exec.add_node(node);

  // Spin executor to process callbacks
  exec.spin();
  rclcpp::shutdown();
}
