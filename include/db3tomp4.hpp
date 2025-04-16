#ifndef ROSBAG_PLAYER_HPP
#define ROSBAG_PLAYER_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <thread>
#include <atomic>
            #include <chrono> 


class RosbagPlayer : public rclcpp::Node {
public:
    RosbagPlayer();
    ~RosbagPlayer();

private:
    void play_bag();
    
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rosbag2_cpp::Reader reader_;
    std::shared_ptr<std::thread> processing_thread_;
    std::string input_path;
    std::string output_path_;
    std::string output_name_;
    std::string topic_name_;
};

#endif // ROSBAG_PLAYER_HPP
