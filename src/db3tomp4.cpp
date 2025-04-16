#include "db3tomp4.hpp"
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <signal.h>

std::atomic<bool> stop_playback(false);

void on_exit([[maybe_unused]] int sig)
{
    RCUTILS_LOG_INFO("Exit by Ctrl+C");
    stop_playback = true;
}

RosbagPlayer::RosbagPlayer() : Node("rosbag_player_node")
{
    // Declare parameters for input, output paths, and topic name
    input_path = this->declare_parameter<std::string>("input_path", "");
    output_path_ = this->declare_parameter<std::string>("output_path", "");
    topic_name_ = this->declare_parameter<std::string>("topic_name", "/image_raw/compressed");
    output_name_ = this->declare_parameter<std::string>("output_name", "output.mp4");

    // Initialize publisher
    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera_image", rclcpp::SensorDataQoS());

    // Set up signal handler
    signal(SIGINT, on_exit);

    // Open the rosbag file
    reader_.open(input_path);

    // Start a new thread for bag playback
    processing_thread_ = std::make_shared<std::thread>(&RosbagPlayer::play_bag, this);
}

RosbagPlayer::~RosbagPlayer()
{
    if (processing_thread_ && processing_thread_->joinable())
    {
        processing_thread_->join();
    }
}

void RosbagPlayer::play_bag()
{
    RCLCPP_INFO(this->get_logger(), "Starting bag playback.");
    cv::startWindowThread();
    cv::namedWindow("bag_image", cv::WINDOW_KEEPRATIO);

    while (rclcpp::ok() && !stop_playback)
    {
        try
        {
            auto start_time = std::chrono::high_resolution_clock::now();
            auto bag_message = reader_.read_next();
            if (bag_message->topic_name == topic_name_)
            {
                process_bag_message(*bag_message);
            }
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
            if (duration < 100 && duration > 1)
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100) - std::chrono::milliseconds(duration)); // Fix the sleep_for call
            }
        }
        catch (const std::runtime_error &e)
        {
            handle_bag_error(e);
        }
    }

    handle_bag_end();
}

void RosbagPlayer::process_bag_message(const rosbag2_storage::SerializedBagMessage &bag_message)
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

    if (first_frame_)
    {
        first_frame_ = false;
        videowriter_.open(output_name_, cv::VideoWriter::fourcc('a', 'v', 'c', '1'), 60, img_.size(), true);
        if (!videowriter_.isOpened())
        {
            RCLCPP_ERROR(this->get_logger(), "Error opening video writer.");
            rclcpp::shutdown();
        }
    }

    videowriter_.write(img_);
    cv::waitKey(1);
}

void RosbagPlayer::handle_bag_end()
{
    RCLCPP_INFO(this->get_logger(), "No more messages in the bag.");
    RCLCPP_INFO(this->get_logger(), "Saving video... Please wait...");
    videowriter_.release();
    cv::destroyAllWindows();
    RCLCPP_INFO(this->get_logger(), "Automatically saved the video and exiting!");
    rclcpp::shutdown();
}

void RosbagPlayer::handle_bag_error(const std::runtime_error &e)
{
    if (e.what() == std::string("Bag is at end. No next message."))
    {
        RCLCPP_INFO(this->get_logger(), "Bag file reached end. Stopping playback.");
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Unexpected error: %s", e.what());
        rclcpp::shutdown();
    }
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    // Create the rosbag player and start playback
    rclcpp::spin(std::make_shared<RosbagPlayer>());

    // Shutdown ROS
    rclcpp::shutdown();

    return 0;
}
