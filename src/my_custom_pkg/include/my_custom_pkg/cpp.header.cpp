#include "rclcpp/rclcpp.hpp"

class MyCustomNode : public rclcpp::Node
{
    public:
        MyCustomNode(): Node("my_cpp_node"){
            RCLCPP_INFO(this->get_logger(), "TEST Cpp NODE");
        }

    private:
};