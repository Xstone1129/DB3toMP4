#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <string>
#include <vector>

class RosbagPlayer : public rclcpp::Node {
public:
    RosbagPlayer();
    ~RosbagPlayer();

private:
    void run_all_files();                                 // 批量运行所有文件
    void process_file(const std::string &file_path);      // 处理单个 db3 文件
    void process_bag_message(const rosbag2_storage::SerializedBagMessage &bag_message, const std::string &output_name);

    std::shared_ptr<std::thread> processing_thread_;
    rosbag2_cpp::readers::SequentialReader reader_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    cv::Mat img_;
    cv::VideoWriter videowriter_;

    std::string input_path_;
    std::string output_path_;

    std::string topic_name_;
    std::vector<std::string> db3_files_;
    bool first_frame_ = true;
};
