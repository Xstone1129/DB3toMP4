#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp>
#include <thread>
#include <atomic>

using namespace std::chrono_literals;

std::atomic<bool> stop_playback(false);

void on_exit([[maybe_unused]] int sig) {
    RCUTILS_LOG_INFO("Exit by Ctrl+C");
    stop_playback = true;
}

class RosbagPlayer : public rclcpp::Node {
public:
    RosbagPlayer(const rclcpp::NodeOptions & options)
        : Node("rosbag_player_node", options) {
        this->declare_parameter<std::string>("rosbag_file", "");
        this->get_parameter("rosbag_file", rosbag_file);

        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("camera_image", rclcpp::SensorDataQoS());
        signal(SIGINT, on_exit);
        // 创建一个新的线程来处理bag文件
        reader_.open(rosbag_file);

        processing_thread_ = std::make_shared<std::thread>(&RosbagPlayer::play_bag, this);
    }

    ~RosbagPlayer() {
        if (processing_thread_ && processing_thread_->joinable()) {
            processing_thread_->join();
        }
    }

private:
    void play_bag() {
        RCLCPP_INFO(this->get_logger(), "Starting bag playback.");
        cv::startWindowThread();
        cv::namedWindow("bag_image", cv::WINDOW_KEEPRATIO);
        cv::VideoWriter videowriter;

        bool first_frame = true;
        cv::Mat img;

        while (rclcpp::ok() && !stop_playback) {
            try {
                auto start_time = std::chrono::high_resolution_clock::now();
                auto bag_message = reader_.read_next();
                auto ros_time = rclcpp::Clock().now();  

                // 处理 CompressedImage 消息
                if (bag_message->topic_name == "/image_raw/compressed") {
                    auto image_msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
                    rclcpp::Serialization<sensor_msgs::msg::CompressedImage> serialization;
                    rclcpp::SerializedMessage serialized_msg(*bag_message->serialized_data);
                    serialization.deserialize_message(&serialized_msg, image_msg.get());
                    img = cv::imdecode(image_msg->data, cv::IMREAD_COLOR);
                    auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img).toImageMsg();
                    msg->header.stamp = ros_time;
                    image_publisher_->publish(*msg);
                    cv::imshow("bag_image", img);

                    if (first_frame) {
                        first_frame = false;
                        videowriter.open("output1.mp4", cv::VideoWriter::fourcc('a', 'v', 'c', '1'), 60, img.size(), true);
                        if (!videowriter.isOpened()) {
                            RCLCPP_ERROR(this->get_logger(), "Error opening video writer.");
                            rclcpp::shutdown();
                        }
                    }

                    videowriter.write(img);
                    cv::waitKey(1);
                }

                auto end_time = std::chrono::high_resolution_clock::now();
                auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count();
                if ((duration < 100) && (duration > 1)) {
                    std::this_thread::sleep_for(100ms - std::chrono::milliseconds(duration));
                }
            } catch (const std::runtime_error& e) {
                if (e.what() == std::string("Bag is at end. No next message.")) {
                    RCLCPP_INFO(this->get_logger(), "Bag file reached end. Stopping playback.");
                    break;
                } else {
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

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rosbag2_cpp::Reader reader_;
    std::shared_ptr<std::thread> processing_thread_;
    std::string rosbag_file;
};

RCLCPP_COMPONENTS_REGISTER_NODE(RosbagPlayer)