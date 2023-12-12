#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "yolov8_msgs/msg/detection.hpp"
#include "yolov8_msgs/msg/detection_array.hpp"
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


    void depthImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        float depth = cv_ptr->image.at<float>(300, 300);
        RCLCPP_INFO(get_logger(), "Image Received. Depth at pixel (10, 30): %f", depth);
        cv::putText(cv_ptr->image,"depth: "+ std::to_string(depth), cv::Point(300,300),cv::FONT_HERSHEY_DUPLEX,1.0,CV_RGB(118,0,0),2);
        depth_pub_.publish(cv_ptr->toImageMsg());

    }

    void rgbImageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGRA8);
        rgb_pub_.publish(cv_ptr->toImageMsg());

    }

    void detectionCallback([[maybe_unused]] const yolov8_msgs::msg::DetectionArray::ConstSharedPtr& msg)
    {
        
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