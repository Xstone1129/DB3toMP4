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
    input_path = this->declare_parameter<std::string>("input_path", ""); 
    output_path_ = this->declare_parameter<std::string>("output_path", ""); 
    topic_name_ = this->declare_parameter<std::string>("topic_name", "/image_raw/compressed"); 
    output_name_ = this->declare_parameter<std::string>("output_name", "output.mp4"); 
    image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera_image", rclcpp::SensorDataQoS());
    signal(SIGINT, on_exit);
    reader_.open(input_path);

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
    cv::VideoWriter videowriter;

    bool first_frame = true;
    cv::Mat img;

    while (rclcpp::ok() && !stop_playback)
    {
        try
        {
            auto start_time = std::chrono::high_resolution_clock::now();
            auto bag_message = reader_.read_next();
            auto ros_time = rclcpp::Clock().now();

            if (bag_message->topic_name == "topic_name_")
            {
                auto image_msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
                rclcpp::Serialization<sensor_msgs::msg::CompressedImage> serialization;
                rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
                serialization.deserialize_message(&serialized_msg, image_msg.get());
                img = cv::imdecode(image_msg->data, cv::IMREAD_COLOR);
                auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
                msg->header.stamp = ros_time;
                image_publisher_->publish(*msg);
                cv::imshow("bag_image", img);

                if (first_frame)
                {
                    first_frame = false;
                    videowriter.open(output_name_, cv::VideoWriter::fourcc('a', 'v', 'c', '1'), 60, img.size(), true);
                    if (!videowriter.isOpened())
                    {
                        RCLCPP_ERROR(this->get_logger(), "Error opening video writer.");
                        rclcpp::shutdown();
                    }
                }

                videowriter.write(img);
                cv::waitKey(1);
            }


            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
            if ((duration < 100) && (duration > 1))
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(100) - std::chrono::milliseconds(duration)); // Fix the sleep_for call
            }
        }
        catch (const std::runtime_error &e)
        {
            if (e.what() == std::string("Bag is at end. No next message."))
            {
                RCLCPP_INFO(this->get_logger(), "Bag file reached end. Stopping playback.");
                break;
            }
            else
            {
                RCLCPP_ERROR(this->get_logger(), "Unexpected error: %s", e.what());
                rclcpp::shutdown();
            }
        }
    }

    RCLCPP_INFO(this->get_logger(), "No more messages in the bag.");
    RCLCPP_INFO(this->get_logger(), "Saving video ... Plz wait ...");
    videowriter.release();
    cv::destroyAllWindows();
    RCLCPP_INFO(this->get_logger(), "Automatically save the video and exit!");
    rclcpp::shutdown();
}

int main(int argc, char **argv)
{
    // 2.初始化ROS2客户端；
    rclcpp::init(argc, argv);
    // 4.调用spin函数，传入自定义类对象指针；
    rclcpp::spin(std::make_shared<RosbagPlayer>());
    // 5.释放资源
    rclcpp::shutdown();

    return 0;
}