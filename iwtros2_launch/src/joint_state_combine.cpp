#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

using namespace std::placeholders;

class JointStateUpdater: public rclcpp::Node
{
    public:
        JointStateUpdater(const rclcpp::NodeOptions options): Node("joint_state_updater_node", options)
        {
            this->_sub = this->create_subscription<sensor_msgs::msg::JointState>("joint_states", 10, std::bind(&JointStateUpdater::callback, this, _1));
            this->_pub = this->create_publisher<sensor_msgs::msg::JointState>("~/joint_states", 10);
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

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    return 0;
}