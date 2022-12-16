#include "iwtros2_launch/create_motion_planning.hpp"

#include <moveit/kinematic_constraints/utils.h>
#include <moveit/robot_state/conversions.h>

namespace iwtros2
{
CreateMotion::CreateMotion(const rclcpp::Node::SharedPtr &node,
                           const std::shared_ptr<moveit::planning_interface::MoveGroupInterface> &group)
    : _node(node), _group(group)
{
    // Planning Pipeline Components
    this->_robot_model_loader.reset(new robot_model_loader::RobotModelLoader(_node, "robot_description"));
    this->_psm.reset(new planning_scene_monitor::PlanningSceneMonitor(_node, _robot_model_loader));
    _psm->startSceneMonitor();
    _psm->startWorldGeometryMonitor();
    _psm->startStateMonitor();

    this->_robot_model = _robot_model_loader->getModel();
    this->_robot_state.reset(
        new moveit::core::RobotState(planning_scene_monitor::LockedPlanningSceneRO(_psm)->getCurrentState()));
    this->_joint_model_group = _robot_state->getJointModelGroup(conf.ARM_GROUP_NAME);
}

bool CreateMotion::joint_space_goal(const std::vector<double> &joint_values,
                                    const moveit::core::RobotStatePtr robot_state,
                                    moveit::planning_interface::MoveGroupInterface::Plan &plan)
{
    planning_pipeline::PlanningPipelinePtr planner_pipeline(new planning_pipeline::PlanningPipeline(
        _robot_model, _node, conf.PIPELINE_ID, "planning_plugin", "request_adapters"));

    _robot_state = _group->getCurrentState(5.0);
    moveit_msgs::msg::RobotState start_state;
    moveit::core::robotStateToRobotStateMsg(*_robot_state, start_state);

    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;

    moveit::core::RobotState goal_state(*_robot_state);
    goal_state.setJointGroupPositions(_joint_model_group, joint_values);

    req.group_name = conf.ARM_GROUP_NAME;
    req.pipeline_id = conf.PIPELINE_ID;
    req.planner_id = conf.PTP_PLANNER_ID;
    req.start_state = start_state;
    req.max_velocity_scaling_factor = 0.05;
    req.max_acceleration_scaling_factor = 1.0;

    moveit_msgs::msg::Constraints joint_space_goal =
        kinematic_constraints::constructGoalConstraints(goal_state, _joint_model_group);
    req.goal_constraints.push_back(joint_space_goal);

    {
        planning_scene_monitor::LockedPlanningSceneRO lscene(_psm);
        planner_pipeline->generatePlan(lscene, req, res);
    }

    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
        RCLCPP_ERROR(_node->get_logger(), "Could not compute plan successfully");
        return false;
    }

    moveit_msgs::msg::MotionPlanResponse response;
    res.getMessage(response);
    plan.planning_time_ = response.planning_time;
    plan.start_state_ = response.trajectory_start;
    plan.trajectory_ = response.trajectory;
    return true;
}

bool CreateMotion::pose_goal(const geometry_msgs::msg::PoseStamped &pose, const moveit::core::RobotStatePtr robot_state,
                             moveit::planning_interface::MoveGroupInterface::Plan &plan, const bool is_linear = false)
{
    planning_pipeline::PlanningPipelinePtr planner_pipeline(new planning_pipeline::PlanningPipeline(
        _robot_model, _node, conf.PIPELINE_ID, "planning_plugin", "request_adapters"));

    std::vector<double> position_tolerance(3, 0.01f);
    std::vector<double> orientation_tolerance(3, 0.01f);

    _robot_state = _group->getCurrentState(5.0);
    moveit_msgs::msg::RobotState start_state;
    moveit::core::robotStateToRobotStateMsg(*_robot_state, start_state);

    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;

    req.group_name = conf.ARM_GROUP_NAME;
    req.pipeline_id = conf.PIPELINE_ID;

    if (is_linear)
        req.planner_id = conf.LIN_PLANNER_ID;
    else
        req.planner_id = conf.PTP_PLANNER_ID;

    req.start_state = start_state;
    req.max_velocity_scaling_factor = 0.05;
    req.max_acceleration_scaling_factor = 1.0;

    moveit_msgs::msg::Constraints pose_goal = kinematic_constraints::constructGoalConstraints(
        conf.ARM_END_EFFECTOR, pose, position_tolerance, orientation_tolerance);
    req.goal_constraints.push_back(pose_goal);

    {
        planning_scene_monitor::LockedPlanningSceneRO lscene(_psm);
        planner_pipeline->generatePlan(lscene, req, res);
    }

    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
        RCLCPP_ERROR(_node->get_logger(), "Could not compute plan successfully");
        return false;
    }

    moveit_msgs::msg::MotionPlanResponse response;
    res.getMessage(response);
    plan.planning_time_ = response.planning_time;
    plan.start_state_ = response.trajectory_start;
    plan.trajectory_ = response.trajectory;
    return true;
}
} // namespace iwtros2
