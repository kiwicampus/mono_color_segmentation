#ifndef IMAGE_PROCESSOR_NODE_HPP_
#define IMAGE_PROCESSOR_NODE_HPP_

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "vision_msgs/msg/label_info.hpp"
#include "vision_msgs/msg/vision_class.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <opencv4/opencv2/core.hpp>

class MonoColorSegmentation : public rclcpp::Node {
public:
    MonoColorSegmentation(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg);
    void createMask(const sensor_msgs::msg::Image::SharedPtr& image);
    cv::Scalar target_color_;
    vision_msgs::msg::LabelInfo label_info_;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr mask_publisher_;
    rclcpp::Publisher<vision_msgs::msg::LabelInfo>::SharedPtr label_info_publisher_;
};

#endif  // IMAGE_PROCESSOR_NODE_HPP_
