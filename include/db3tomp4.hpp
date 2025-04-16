#ifndef DB3TOMP4_HPP
#define DB3TOMP4_HPP

#include <atomic>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <thread>

class RosbagPlayer : public rclcpp::Node
{
public:
    RosbagPlayer();
    ~RosbagPlayer();

private:
    void play_bag();
    void process_bag_message(const rosbag2_storage::SerializedBagMessage &bag_message);
    void handle_bag_end();
    void handle_bag_error(const std::runtime_error &e);

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rosbag2_cpp::Reader reader_;
    std::shared_ptr<std::thread> processing_thread_;

    std::string input_path;
    std::string output_path_;
    std::string topic_name_;
    std::string output_name_;

    cv::VideoWriter videowriter_;
    cv::Mat img_;
    bool first_frame_ = true;

    std::atomic<bool> stop_playback;
};

#endif // DB3TOMP4_HPP
