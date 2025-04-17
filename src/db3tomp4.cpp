#include "db3tomp4.hpp"
#include <chrono>
#include <cv_bridge/cv_bridge.h>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/serialization.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <signal.h>

namespace fs = std::filesystem;

std::atomic<bool> stop_playback(false);

void on_exit([[maybe_unused]] int sig)
{
    RCUTILS_LOG_INFO("Exit by Ctrl+C");
    stop_playback = true;
}

RosbagPlayer::RosbagPlayer() : Node("rosbag_player_node")
{
    input_path_ = this->declare_parameter<std::string>("input_path", "/home/xu/视频/record/UAV_auto_record");
    topic_name_ = this->declare_parameter<std::string>("topic_name", "/image_raw/compressed");
    output_path_ = this->declare_parameter<std::string>("output_path", "/home/xu/视频/record/output"); // 新增输出路径参数

    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera_image", rclcpp::SensorDataQoS());
    signal(SIGINT, on_exit);

    if (fs::is_directory(input_path_))
    {
        for (const auto &entry : fs::directory_iterator(input_path_))
        {
            if (entry.path().extension() == ".db3")
            {
                db3_files_.push_back(entry.path().string());
            }
        }
        std::sort(db3_files_.begin(), db3_files_.end());
    }
    else if (fs::is_regular_file(input_path_) && input_path_.size() >= 4 && input_path_.substr(input_path_.size() - 4) == ".db3")
    {
        db3_files_.push_back(input_path_);
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid input path: %s", input_path_.c_str());
        rclcpp::shutdown();
        return;
    }

    processing_thread_ = std::make_shared<std::thread>(&RosbagPlayer::run_all_files, this);
}

void RosbagPlayer::process_file(const std::string &file_path)
{
    RCLCPP_INFO(this->get_logger(), "Processing file: %s", file_path.c_str());

    // 确保输出路径存在，如果不存在则创建
    if (!fs::exists(output_path_))
    {
        try
        {
            fs::create_directories(output_path_);
            RCLCPP_INFO(this->get_logger(), "Created output directory: %s", output_path_.c_str());
        }
        catch (const fs::filesystem_error &e)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to create output directory: %s", e.what());
            return;
        }
    }

    rosbag2_storage::StorageOptions storage_options;
    storage_options.uri = file_path;
    storage_options.storage_id = "sqlite3";

    rosbag2_cpp::ConverterOptions converter_options;
    converter_options.input_serialization_format = "cdr";
    converter_options.output_serialization_format = "cdr";

    try
    {
        reader_.open(storage_options, converter_options);
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to open rosbag: %s", e.what());
        return;
    }

    // 生成输出文件路径，使用自定义的输出路径
    std::string output_name = (fs::path(output_path_) / fs::path(file_path).stem()).string() + ".mp4";
    first_frame_ = true;

    cv::startWindowThread();
    cv::namedWindow("bag_image", cv::WINDOW_KEEPRATIO);

    while (rclcpp::ok() && !stop_playback)
    {
        try
        {
            auto bag_message = reader_.read_next();
            if (bag_message->topic_name == topic_name_)
            {
                process_bag_message(*bag_message, output_name);
            }
        }
        catch (const std::runtime_error &e)
        {
            if (std::string(e.what()) == "Bag is at end. No next message.")
            {
                break;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Unexpected error: %s", e.what());
                break;
            }
        }
    }

    videowriter_.release();
    cv::destroyAllWindows();
    RCLCPP_INFO(this->get_logger(), "Saved video: %s", output_name.c_str());
}

RosbagPlayer::~RosbagPlayer()
{
    if (processing_thread_ && processing_thread_->joinable())
    {
        processing_thread_->join();
    }
}

void RosbagPlayer::run_all_files()
{
    for (const auto &file : db3_files_)
    {
        if (stop_playback)
            break;
        process_file(file);
    }
    rclcpp::shutdown();
}


void RosbagPlayer::process_bag_message(const rosbag2_storage::SerializedBagMessage &bag_message, const std::string &output_name)
{
    auto ros_time = rclcpp::Clock().now();

    auto image_msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
    rclcpp::Serialization<sensor_msgs::msg::CompressedImage> serialization;
    rclcpp::SerializedMessage serialized_msg(*bag_message.serialized_data);
    serialization.deserialize_message(&serialized_msg, image_msg.get());

    img_ = cv::imdecode(image_msg->data, cv::IMREAD_COLOR);
    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img_).toImageMsg();
    msg->header.stamp = ros_time;
    image_publisher_->publish(*msg);

    cv::imshow("bag_image", img_);
    cv::waitKey(1);

    if (first_frame_)
    {
        first_frame_ = false;
        videowriter_.open(output_name, cv::VideoWriter::fourcc('a', 'v', 'c', '1'), 30, img_.size(), true);
        if (!videowriter_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Error opening video writer for: %s", output_name.c_str());
            stop_playback = true;
            return;
        }
    }

    videowriter_.write(img_);
}
int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RosbagPlayer>());
    rclcpp::shutdown();
    return 0;
}