#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::placeholders;

class JointStateUpdater : public rclcpp::Node
{
  public:
    JointStateUpdater() : Node("joint_state_updater_node")
    {
        this->_sub = this->create_subscription<sensor_msgs::msg::JointState>(
            "joint_states", 10, std::bind(&JointStateUpdater::callback, this, _1));
        this->_pub = this->create_publisher<sensor_msgs::msg::JointState>("/move_group/joint_states", 10);
        RCLCPP_INFO(this->get_logger(), "Joint State Updater for Simulation Starts");
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr _sub;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr _pub;

    void callback(const sensor_msgs::msg::JointState::SharedPtr data) const
    {
        sensor_msgs::msg::JointState joint;
        joint.header.stamp = rclcpp::Clock().now();
        joint.name = data->name;
        joint.position = data->position;
        joint.velocity = data->velocity;
        joint.effort = data->effort;

        _pub->publish(joint);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointStateUpdater>();
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}