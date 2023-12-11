#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/image.hpp"



class DepthPublisherNode : public rclcpp::Node
{
public:
    DepthPublisherNode() : Node("depth_publisher_node"),
    node_handle_(std::shared_ptr<DepthPublisherNode>(this, [](auto *){})),
    image_transport_(node_handle_),
    image_pub_(image_transport_.advertise("/marked_images/image_raw", 2))
    {   
        // rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        //auto qos = rclcpp::QoS(QoSInitialization(qos_profile.history, 1), qos_profile)
        RCLCPP_INFO(get_logger(), "Before subscribing");
        image_sub_ = image_transport_.subscribe("/depth_to_rgb/image_raw", 10, std::bind(&DepthPublisherNode::imageCallback, this, std::placeholders::_1));
        RCLCPP_INFO(get_logger(), "After subscribing");
    }

private:
    rclcpp::Node::SharedPtr node_handle_;
    image_transport::ImageTransport image_transport_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;
    
    // Corrected imageCallback signature
    void imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr& msg)
    {
        RCLCPP_INFO(get_logger(), "Subscribed!");
        cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
        float depth = cv_ptr->image.at<float>(300, 300);
        RCLCPP_INFO(get_logger(), "Image Received. Depth at pixel (10, 30): %f", depth);
        cv::putText(cv_ptr->image, "hello", cv::Point(300,300),cv::FONT_HERSHEY_DUPLEX,1.0,CV_RGB(118,0,0),2);
        image_pub_.publish(cv_ptr->toImageMsg());

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