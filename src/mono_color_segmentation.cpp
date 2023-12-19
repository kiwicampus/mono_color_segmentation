#include "mono_color_segmentation/mono_color_segmentation.hpp"

MonoColorSegmentation::MonoColorSegmentation(const rclcpp::NodeOptions & options) 
    : rclcpp::Node("image_processor_node", options) {
    std::vector<long int> target_color_array{155, 155, 155}; // the color of gazebo's ground plane
    this->declare_parameter("target_color", target_color_array);
    this->get_parameter("target_color", target_color_array);
    target_color_ = cv::Scalar(target_color_array[2], target_color_array[1], target_color_array[0]);

    image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw", 10, std::bind(&MonoColorSegmentation::imageCallback, this, std::placeholders::_1));

    mask_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("mask", 10);
    rclcpp::QoS label_info_qos(rclcpp::KeepLast(10));
    label_info_qos.transient_local();
    label_info_publisher_ = this->create_publisher<vision_msgs::msg::LabelInfo>(
        "label_info", label_info_qos);

    label_info_.class_map.resize(2);
    label_info_.class_map.at(0).class_name = "others";
    label_info_.class_map.at(0).class_id = 0;
    label_info_.class_map.at(1).class_name = "ground_plane";
    label_info_.class_map.at(1).class_id = 255;
    label_info_publisher_->publish(label_info_);
}

void MonoColorSegmentation::imageCallback(const sensor_msgs::msg::Image::SharedPtr msg) {
    createMask(msg);
}

void MonoColorSegmentation::createMask(const sensor_msgs::msg::Image::SharedPtr& image) {
    // Convert the ROS image message to an OpenCV image
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(image);
    } catch (cv_bridge::Exception& e) {
        RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
        return;
    }

    // Create a binary mask of the same size as the input image
    cv::Mat mask;
    cv::inRange(cv_ptr->image, target_color_, target_color_, mask);

    // Convert the mask to a ROS image message
    cv_bridge::CvImage mask_msg;
    mask_msg.header = image->header;  // Use the same header as the input image
    mask_msg.encoding = sensor_msgs::image_encodings::MONO8;
    mask_msg.image = mask;

    // Publish the mask
    mask_publisher_->publish(*mask_msg.toImageMsg());
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(MonoColorSegmentation)
