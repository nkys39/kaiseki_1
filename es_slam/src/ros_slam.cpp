
#include "es_slam/es_slam_node.hpp"
#include <rclcpp/rclcpp.hpp>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto es_slam_node = std::make_shared<EsSlamNode>();
    rclcpp::spin(es_slam_node);
    rclcpp::shutdown();
    return 0;
}
