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
    std::thread([&executor]() { executor->spin(); }).detach();
    executor_g->add_node(node_g);

    // Setup Move group planner
    auto group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "iiwa_arm");
    group->setPlanningPipelineId("pilz");
    group->setPlannerId("PTP");
    group->setMaxVelocityScalingFactor(0.1);
    group->setMaxAccelerationScalingFactor(0.2);
    group->setPoseReferenceFrame("iiwa7_link_0");
    group->setEndEffector("iiwa7_link_7");
    group->allowReplanning(true);

    auto iiwa_move = std::make_shared<iwtros2::IiwaMove>(node, group);
    auto plc_contl = std::make_shared<iwtros2::ControlPLC>(node_g);

    geometry_msgs::msg::PoseStamped table_pose_0 = 
        iiwa_move->generatePose(-0.141, 0.585, 1.265, M_PI, 0, 3 * M_PI / 4, "iiwa7_link_0");
    geometry_msgs::msg::PoseStamped table_pose_1 = 
        iiwa_move->generatePose(0.0530, 0.585, 1.265, M_PI, 0, 3 * M_PI / 4, "iiwa7_link_0");
    geometry_msgs::msg::PoseStamped table_pose_2 = 
        // iiwa_move->generatePose(0.0530, 0.756, 1.265, M_PI, 0, 3 * M_PI / 4, "iiwa7_link_0");
        iiwa_move->generatePose(0.0515, 0.756, 1.265, M_PI, 0, 3 * M_PI / 4, "iiwa7_link_0");
    geometry_msgs::msg::PoseStamped table_pose_3 = 
        iiwa_move->generatePose(-0.1423, 0.761, 1.265, M_PI, 0, 3 * M_PI / 4, "iiwa7_link_0");
    geometry_msgs::msg::PoseStamped home_pose =
        iiwa_move->generatePose(0.5, 0, 1.65896, -M_PI, 0, M_PI, "iiwa7_link_0");
    geometry_msgs::msg::PoseStamped conveyor_pose =
        // iiwa_move->generatePose(0.235, -0.43, 1.263, M_PI, 0, M_PI / 4, "iiwa7_link_0"); // default
        // iiwa_move->generatePose(0.2370, -0.4298, 1.263, M_PI, 0, M_PI / 4, "iiwa7_link_0");
        iiwa_move->generatePose(0.2360, -0.4298, 1.263, M_PI, 0, M_PI / 4, "iiwa7_link_0");
    geometry_msgs::msg::PoseStamped hochregallager_pose =
        iiwa_move->generatePose(0.555, 0.069, 1.345, M_PI, 0, 3 * M_PI / 4, "iiwa7_link_0"); 
    geometry_msgs::msg::PoseStamped loading_pose =
        iiwa_move->generatePose(0.0, 0.5, 1.245, M_PI, 0, 3 * M_PI / 4, "iiwa7_link_0");

    geometry_msgs::msg::PoseStamped slot_pose;
    bool use_table = true;
    

    rclcpp::Rate rate(1);
    executor_g->spin_once();
    while (rclcpp::ok())
    {
        if (plc_contl->conveyor_pick || plc_contl->table_pick){
            switch (plc_contl->slot_id)
            {
                case 0:
                    slot_pose = table_pose_0;
                    break;
                case 1:
                    slot_pose = table_pose_1;
                    break;
                case 2:
                    slot_pose = table_pose_2;
                    break;
                case 3:
                    slot_pose = table_pose_3;
                    break;
                default:
                    RCLCPP_ERROR(rclcpp::get_logger("iiwa_motion_controller_node"), "Invalid slot ID!");
                    plc_contl->conveyor_pick = false;
                    plc_contl->table_pick = false;
            }
        }

        if (plc_contl->move_home)
        {
            iiwa_move->go_home(false);
            plc_contl->move_home = false;
            plc_contl->plc_publish(true, false, false, false);
        }
        if (plc_contl->conveyor_pick)
        {
            iiwa_move->pnpPipeLine(conveyor_pose, table_pose_0, 0.15, false, true); //hochregallager_pose
            plc_contl->conveyor_pick = false;
            plc_contl->plc_publish(false, false, false, true); // Placed the product on Hochregallager
        }
        if (plc_contl->hochregallager_pick)
        {
            iiwa_move->pick_action(hochregallager_pose, 0.15);
            plc_contl->plc_publish(false, true, false, false);
            iiwa_move->go_home(false);
            iiwa_move->place_action(conveyor_pose, 0.15);
            // iiwa_move->pnpPipeLine(hochregallager_pose, conveyor_pose, 0.15, false, false);
            plc_contl->hochregallager_pick = false;
            plc_contl->plc_publish(false, false, true, false); // Placed the product on conveyor belt.
        }
        else
        {
            RCLCPP_INFO(rclcpp::get_logger("iiwa_motion_controller_node"), "Doing nothing and I am Sad!");
        }

        // executor->spin_once();
        executor_g->spin_once();
        rate.sleep();
    }

    rclcpp::shutdown();

    return 0;
}
