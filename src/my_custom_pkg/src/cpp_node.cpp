#include "rclcpp/rclcpp.hpp"
#include "my_custom_pkg/cpp.header.cpp"

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyCustomNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}