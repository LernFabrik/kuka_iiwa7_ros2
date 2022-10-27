#include <iwtros2_launch/gripper_controller.hpp>
#include <iwtros2_launch/iiwa_manipulation.hpp>

using GripperCommand = control_msgs::action::GripperCommand;
using ClientGoalHandle = rclcpp_action::ClientGoalHandle<GripperCommand>;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("iiwa_motion_controller_node", options);
    auto node_g = rclcpp::Node::make_shared("plc_control_sub_pub_node", options);

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    auto executor_g = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(node);
    executor_g->add_node(node_g);

    // Setup Move group planner
    auto group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "iiwa_arm");
    group->setPlanningPipelineId("pilz");
    group->setPlannerId("PTP");
    group->setMaxVelocityScalingFactor(0.1);
    group->setMaxAccelerationScalingFactor(0.2);
    group->setPoseReferenceFrame("iiwa7_link_0");
    group->setEndEffector("iiwa7_tool0");
    group->allowReplanning(true);

    auto iiwa_move = std::make_shared<iwtros2::IiwaMove>(node, group, executor);
    auto plc_contl = std::make_shared<iwtros2::ControlPLC>(node_g);

    geometry_msgs::msg::PoseStamped home_pose =
        iiwa_move->generatePose(0.5, 0, 1.61396, -M_PI, 0, M_PI, "iiwa7_link_0");
    geometry_msgs::msg::PoseStamped conveyor_pose =
        iiwa_move->generatePose(0.235, -0.43, 1.218, M_PI, 0, M_PI / 4, "iiwa7_link_0"); // 1.221
    geometry_msgs::msg::PoseStamped hochregallager_pose =
        iiwa_move->generatePose(0.551, 0.0635, 1.30, M_PI, 0, 3 * M_PI / 4, "iiwa7_link_0");
    geometry_msgs::msg::PoseStamped loading_pose =
        iiwa_move->generatePose(0.0, 0.5, 1.2, M_PI, 0, 3 * M_PI / 4, "iiwa7_link_0");

    rclcpp::Rate rate(1);
    while (rclcpp::ok())
    {

        if (plc_contl->move_home)
        {
            iiwa_move->go_home(false);
            plc_contl->move_home = false;
            plc_contl->plc_publish(true, false, false);
        }
        if (plc_contl->conveyor_pick)
        {
            iiwa_move->pnpPipeLine(conveyor_pose, hochregallager_pose, 0.15, false);
            plc_contl->conveyor_pick = false;
            plc_contl->plc_publish(false, false, true);
        }
        if (plc_contl->hochregallager_pick)
        {
            iiwa_move->pnpPipeLine(hochregallager_pose, conveyor_pose, 0.15, false);
            plc_contl->hochregallager_pick = false;
            plc_contl->plc_publish(false, true, false);
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("iiwa_motion_controller_node"), "Doing nothing and I am Sad!");
        }

        executor->spin_once();
        executor_g->spin_once();
        rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}