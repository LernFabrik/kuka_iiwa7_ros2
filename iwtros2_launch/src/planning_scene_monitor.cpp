#include <pluginlib/class_loader.hpp>

// MoveIt
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_interface/planning_interface.h"
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("motion_planning_pipeline");

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    auto node = rclcpp::Node::make_shared("motion_planning_pipeline_tutorial", node_options);

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread([&executor]() { executor.spin(); }).detach();

    auto group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node, "iiwa_arm");
    group->setPlanningPipelineId("pilz");
    group->setPlannerId("PTP");
    group->setMaxVelocityScalingFactor(0.1);
    group->setMaxAccelerationScalingFactor(0.2);
    group->setPoseReferenceFrame("iiwa7_link_0");
    group->setEndEffector("iiwa7_tool0");
    group->allowReplanning(true);

    robot_model_loader::RobotModelLoaderPtr robo_model_loader(new robot_model_loader::RobotModelLoader(node, "robot_description"));
    planning_scene_monitor::PlanningSceneMonitorPtr psm(new planning_scene_monitor::PlanningSceneMonitor(node, robo_model_loader));
    psm->startSceneMonitor();
    psm->startWorldGeometryMonitor();
    psm->startStateMonitor();

    moveit::core::RobotModelPtr robot_model = robo_model_loader->getModel();
    moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentState()));

    const moveit::core::JointModelGroup * joint_model_group = robot_state->getJointModelGroup("iiwa_arm");
    planning_pipeline::PlanningPipelinePtr planning_pipeline(new planning_pipeline::PlanningPipeline(robot_model, node, "pilz", "planning_plugin", "request_adapters"));

    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools(node, "iiwa7_link_0", "move_group_tutorial", psm);
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "Motion Planning Pipeline Demo", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    moveit::core::RobotState goal_state(*robot_state);
    std::vector<double> joint_values = { 0.0, 0.0, 0.0, -1.5708, 0.0, 1.5708, 0.0 };

    goal_state.setJointGroupPositions(joint_model_group, joint_values);

    req.group_name = "iiwa_arm";
    req.pipeline_id = "PTP";
    moveit_msgs::msg::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(goal_state, joint_model_group);

    req.goal_constraints.push_back(pose_goal);

    {
        planning_scene_monitor::LockedPlanningSceneRO lscene(psm);
        planning_pipeline->generatePlan(lscene, req, res);
    }

    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
        RCLCPP_ERROR(LOGGER, "Could not compute plan successfully");
        return 0;
    }

    rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_publisher = node->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path", 1);
    moveit_msgs::msg::DisplayTrajectory display_trajectory;

    RCLCPP_INFO(LOGGER, "Visulaize the trajectory");
    moveit_msgs::msg::MotionPlanResponse response;
    res.getMessage(response);

    moveit::planning_interface::MoveGroupInterface::Plan mPlan;
    mPlan.planning_time_ = response.planning_time;
    mPlan.start_state_ = response.trajectory_start;
    mPlan.trajectory_ = response.trajectory;

    display_trajectory.trajectory_start = response.trajectory_start;
    display_trajectory.trajectory.push_back(response.trajectory);

    display_publisher->publish(display_trajectory);
    visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
    visual_tools.trigger();

    /* Wait for user input */
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");

    RCLCPP_INFO(LOGGER, "Done");

    rclcpp::shutdown();
    return 0;
}