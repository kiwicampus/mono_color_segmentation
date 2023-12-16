#include "mono_color_segmentation/mono_color_segmentation.hpp"

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MonoColorSegmentation>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}