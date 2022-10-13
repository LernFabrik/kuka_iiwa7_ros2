#include <iwtros2_launch/iiwa_manipulation.hpp>


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("iiwa_motion_controller_node", options);

    iwtros2::IiwaMove move(node);

    auto executor = std::make_unique<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(node);
    executor->spin();

    rclcpp::shutdown();

    return 0;
}