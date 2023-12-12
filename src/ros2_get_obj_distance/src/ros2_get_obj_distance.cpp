#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "yolov8_msgs/msg/detection.hpp"
#include "yolov8_msgs/msg/point2_d.hpp"
#include "yolov8_msgs/msg/detection_array.hpp"
#include "yolov8_msgs/msg/mask.hpp"
#include "cv_bridge/cv_bridge.h"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"

using namespace std::chrono_literals;

class DepthPublisherNode : public rclcpp::Node
{
public:
    DepthPublisherNode() : Node("depth_publisher_node"),
    node_handle_(std::shared_ptr<DepthPublisherNode>(this, [](auto *){})),
    image_transport_(node_handle_),
    depth_pub_(image_transport_.advertise("/depth/image_marked", 2)),
    rgb_pub_(image_transport_.advertise("/rgb/image_marked", 2))
    {   
        // rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        //auto qos = rclcpp::QoS(QoSInitialization(qos_profile.history, 1), qos_profile)
        depth_sub_ = image_transport_.subscribe("/depth_to_rgb/image_raw", 10, std::bind(&DepthPublisherNode::depthImageCallback, this, std::placeholders::_1));
        rgb_sub_ = image_transport_.subscribe("/rgb/image_raw", 10, std::bind(&DepthPublisherNode::rgbImageCallback, this, std::placeholders::_1));
        mask_sub_ = this->create_subscription<yolov8_msgs::msg::DetectionArray>(
            "/yolo/detections", 10, std::bind(&DepthPublisherNode::detectionCallback, this, std::placeholders::_1));
        
    }

private:
    rclcpp::Node::SharedPtr node_handle_;
    rclcpp::Subscription<yolov8_msgs::msg::DetectionArray>::SharedPtr mask_sub_;
    image_transport::ImageTransport image_transport_;
    image_transport::Subscriber depth_sub_;
    image_transport::Publisher depth_pub_;
    image_transport::Publisher rgb_pub_;
    image_transport::Subscriber rgb_sub_;
    sensor_msgs::msg::Image::SharedPtr depth_image_;
    sensor_msgs::msg::Image::SharedPtr rgb_image_;

    void depthImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        //cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        //float depth = cv_ptr->image.at<float>(300, 300);
        //RCLCPP_INFO(get_logger(), "Image Received. Depth at pixel (10, 30): %f", depth);
        //cv::putText(cv_ptr->image,"depth: "+ std::to_string(depth), cv::Point(300,300),cv::FONT_HERSHEY_DUPLEX,1.0,CV_RGB(118,0,0),2);
        //depth_pub_.publish(cv_ptr->toImageMsg());
        depth_image_ = std::const_pointer_cast<sensor_msgs::msg::Image>(msg);
    }

    void rgbImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        //cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGRA8);
        //rgb_pub_.publish(cv_ptr->toImageMsg());
        rgb_image_ = std::const_pointer_cast<sensor_msgs::msg::Image>(msg);

    }

    void detectionCallback([[maybe_unused]] const yolov8_msgs::msg::DetectionArray& msg)
    {
        float sum_x = 0.0;
        float sum_y = 0.0;
        int num_point = 0; 

        RCLCPP_INFO(get_logger(), "detected");
        
        for (const auto& detection : msg.detections) {
            if (!detection.mask.data.empty()) {
                for (const auto& point : detection.mask.data) {  
                    num_point++;
                    sum_x += point.x;
                    sum_y += point.y;
                }
            }

            float average_x = (num_point > 0) ? sum_x / num_point : 0.0;  // Added a check to avoid division by zero
            float average_y = (num_point > 0) ? sum_y / num_point : 0.0;  // Added a check to avoid division by zero

            if (!(average_x == 0.0 && average_y == 0.0)) {
                RCLCPP_INFO(get_logger(), "Mask received. Centeroid at pixel (%f, %f)",average_x ,average_y);
                if (depth_image_ && rgb_image_) {
                    cv_bridge::CvImageConstPtr cv_depth_ptr = cv_bridge::toCvShare(depth_image_, sensor_msgs::image_encodings::TYPE_32FC1);
                    cv_bridge::CvImageConstPtr cv_rgb_ptr = cv_bridge::toCvShare(rgb_image_, sensor_msgs::image_encodings::BGRA8);
                    cv::Mat image_with_circle = cv_rgb_ptr->image.clone();
                    cv::circle(image_with_circle, cv::Point(static_cast<int>(average_x), static_cast<int>(average_y)), 10, CV_RGB(255, 255, 0), -1);

                    float depth = cv_depth_ptr->image.at<float>(static_cast<int>(average_x), static_cast<int>(average_y));
                    cv::putText(image_with_circle,"depth: "+ std::to_string(depth), cv::Point(average_x,average_y),cv::FONT_HERSHEY_DUPLEX,1.0,CV_RGB(118,0,0),2);
                    depth_pub_.publish(cv_bridge::CvImage(std_msgs::msg::Header(), sensor_msgs::image_encodings::BGRA8, image_with_circle).toImageMsg());
                }   
            }
        }

    }

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<DepthPublisherNode>();

    RCLCPP_INFO(node->get_logger(), "Starting depth publisher node");

    rclcpp::spin(node);
    RCLCPP_INFO(node->get_logger(), "Shutting down depth publisher node");
    rclcpp::shutdown();

    return 0;
}