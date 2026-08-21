#ifndef LOWLEVEL_CAMERA_HPP_
#define LOWLEVEL_CAMERA_HPP_

#include <rclcpp/rclcpp.hpp>
#include "unitree_go/msg/go2_front_video_data.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include <thread>
#include <mutex>
#include <condition_variable>
#include <deque>
#include <atomic>
#include <yaml-cpp/yaml.h>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/frame.h>
#include <libavutil/imgutils.h>
#include <libswscale/swscale.h>
}

namespace go2_lowlevel {
class Camera {
    public:
        explicit Camera(rclcpp::Node::SharedPtr node, const std::string &ns = "",
                 std::vector<rclcpp::Parameter> parameters = {});
        ~Camera();
    private:
        static constexpr std::size_t MAX_QUEUE_SIZE = 20;

        rclcpp::Node::SharedPtr node_;
        rclcpp::QoS qos_{rclcpp::KeepLast(1)};

        rclcpp::Subscription<unitree_go::msg::Go2FrontVideoData>::SharedPtr camera_sub_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
        rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;

        std::thread processing_thread_;
        std::mutex queue_mutex_;
        std::condition_variable queue_cv_;
        std::deque<unitree_go::msg::Go2FrontVideoData::SharedPtr> frame_queue_;
        std::atomic<bool> running_{true};

        const AVCodec* codec_{nullptr};
        AVCodecContext* codec_context_{nullptr};
        AVPacket* packet_{nullptr};
        AVFrame* frame_{nullptr};

        void load_camera_calibration(const std::string &path);
        void camera_callback(const unitree_go::msg::Go2FrontVideoData::SharedPtr msg);
        void processing_loop();

        bool synced_{false};

        sensor_msgs::msg::CameraInfo camera_info_msg_;


};
}  // namespace go2_lowlevel

#endif  // LOWLEVEL_CAMERA_HPP_