#include "iwtros2_launch/iiwa_manipulation.hpp"

#include "moveit_msgs/msg/constraints.hpp"
#include "moveit_msgs/msg/orientation_constraint.hpp"
#include <pluginlib/class_loader.hpp>

// MoveIt
#include <moveit/kinematic_constraints/utils.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

using namespace std::placeholders;

namespace iwtros2
{
/**
 * @brief Construct a new Iiwa Move:: Iiwa Move object
 *
 * @param options ros2 options
 */
IiwaMove::IiwaMove(const rclcpp::Node::SharedPtr &node,
                   const std::shared_ptr<moveit::planning_interface::MoveGroupInterface> &group)
    : _node(node), _group(group)
{
    RCLCPP_INFO(_node->get_logger(), "Move group node is starting");
    // todo initiallize more variables
    // Initialize sub and pub
    // Initialize parameretes

    // Display basic information
    RCLCPP_INFO(_node->get_logger(), "Planning frame: %s", _group->getPlanningFrame().c_str());
    RCLCPP_INFO(_node->get_logger(), "End effector link: %s", _group->getEndEffector().c_str());

    this->_visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools>(
        _node, "iiwa7_link_0", "trajectory_marker", _group->getRobotModel());
    _visual_tools->deleteAllMarkers();
    //_visual_tools->loadRemoteControl();

    this->_text_pose = Eigen::Isometry3d::Identity();
    _text_pose.translation().z() = 1.0;
    _visual_tools->publishText(_text_pose, "MoveGroupInterface_IIWA7", rvt::WHITE, rvt::XLARGE);
    _visual_tools->trigger();

    // Initialize Gripper
    this->_gripper_client = std::make_shared<GripperController>(_node);

    // Creat motion planning
    this->_motion = std::make_shared<CreateMotion>(_node, _group);
}

void IiwaMove::gripper_status_callback(const std_msgs::msg::Bool::SharedPtr result)
{
    RCLCPP_INFO(_node->get_logger(), "Gripper Status received");
    this->_gripper_succeeded = result->data;
}

geometry_msgs::msg::PoseStamped IiwaMove::generatePose(const double x, const double y, const double z,
                                                       const double roll, const double pitch, const double yaw,
                                                       const std::string baselink)
{
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = baselink.c_str();
    pose.header.stamp = rclcpp::Clock().now();

    pose.pose.position.x = x;
    pose.pose.position.y = y;
    pose.pose.position.z = z;

    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    pose.pose.orientation.x = q.x();
    pose.pose.orientation.y = q.y();
    pose.pose.orientation.z = q.z();
    pose.pose.orientation.w = q.w();

    return pose;
}

void IiwaMove::go_home(const bool tmp_pose)
{
    // moveit::core::RobotStatePtr current_state = _group->getCurrentState(10);
    RCLCPP_INFO(_node->get_logger(), "Go Home!");
    _group->setPlannerId("PTP");
    std::vector<double> joint_group_position;
    // current_state->copyJointGroupPositions(joint_model_group, joint_group_position);
    if (tmp_pose)
    {
        joint_group_position = {1.5708, -0.26, 0.0, -1.74533, 0.0, 1.74533, 0.0};
    }
    else
    {
        joint_group_position = {0.0, 0.0, 0.0, -1.5708, 0.0, 1.5708, 0.0};
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan;

    _motion->joint_space_goal(joint_group_position, plan);

    _group->execute(plan);
    // goal_state.setJointGroupPositions("join")
}

void IiwaMove::motionContraints(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> &group)
{
    group->clearPathConstraints();
    moveit_msgs::msg::OrientationConstraint oCon;
    oCon.link_name = "iiwa7_tool0";
    oCon.header.frame_id = "iiwa7_link_0";
    oCon.orientation.w = 1.0;
    oCon.absolute_x_axis_tolerance = 0.1;
    oCon.absolute_y_axis_tolerance = 0.1;
    oCon.absolute_z_axis_tolerance = 0.1;
    oCon.weight = 1.0;

    moveit_msgs::msg::Constraints constraints;
    constraints.orientation_constraints.push_back(oCon);
    group->setPathConstraints(constraints);
}

void IiwaMove::visualMarkers(const geometry_msgs::msg::PoseStamped target_pose,
                             const moveit::planning_interface::MoveGroupInterface::Plan plan, const std::string task)
{
    _visual_tools->deleteAllMarkers();
    RCLCPP_INFO(_node->get_logger(), "Visualizing plan as trajectory line for %s task", task.c_str());
    const moveit::core::JointModelGroup *joint_model_group = _group->getCurrentState()->getJointModelGroup("iiwa_arm");
    _visual_tools->publishAxisLabeled(target_pose.pose, task.c_str());
    _visual_tools->publishText(_text_pose, task.c_str(), rvt::WHITE, rvt::XLARGE);
    _visual_tools->publishTrajectoryLine(plan.trajectory_, joint_model_group);
    _visual_tools->trigger();
}

void IiwaMove::motionExecution(geometry_msgs::msg::PoseStamped pose, const std::string task, const bool is_linear)
{
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (is_linear)
    {
        pose.header.frame_id = "world";
        _motion->pose_goal(pose, plan, is_linear);
    }
    else
    {
        _motion->pose_goal(pose, plan, is_linear);
    }

    _group->execute(plan);
}

void IiwaMove::pnpPipeLine(geometry_msgs::msg::PoseStamped pick, geometry_msgs::msg::PoseStamped place,
                           const double offset, const bool tmp_pose)
{
    RCLCPP_INFO(_node->get_logger(), "IIWA 7 Pre-Pick Pose");
    pick.pose.position.z += offset;
    motionExecution(pick, "Pre-Pick Pose", false);

    RCLCPP_INFO(_node->get_logger(), "IIWA 7 Pre-Pick Pose Gripper OPEN");
    auto pio_future = _gripper_client->open();
    _gripper_client->dead_lock_future(pio_future);
    rclcpp::sleep_for(std::chrono::seconds(2));
    
    RCLCPP_INFO(_node->get_logger(), "IIWA 7 Pick Pose");
    pick.pose.position.z -= offset;
    motionExecution(pick, "Pick Pose", true);

    RCLCPP_INFO(_node->get_logger(), "IIWA 7 Pick Pose Gripper CLOSE");
    auto pic_future = _gripper_client->close();
    _gripper_client->dead_lock_future(pic_future);
    rclcpp::sleep_for(std::chrono::seconds(2));

    RCLCPP_INFO(_node->get_logger(), "IIWA 7 Post Pick Pose");
    pick.pose.position.z += offset;
    motionExecution(pick, "Post Pick Pose", true);

    if (tmp_pose)
        go_home(true);
    else
        go_home(false);

    // Place
    RCLCPP_INFO(_node->get_logger(), "IIWA 7 Pre Place Pose");
    place.pose.position.z += offset;
    motionExecution(place, "Pre Place Pose", false);

    RCLCPP_INFO(_node->get_logger(), "IIWA 7 Place Pose");
    place.pose.position.z -= offset;
    motionExecution(place, "Place Pose", true);

    RCLCPP_INFO(_node->get_logger(), "IIWA 7 Place Pose Gripper OPEN");
    auto plac_future = _gripper_client->open();
    _gripper_client->dead_lock_future(plac_future);
    rclcpp::sleep_for(std::chrono::seconds(2));

    RCLCPP_INFO(_node->get_logger(), "IIWA 7 Post Place Pose");
    place.pose.position.z += offset;
    motionExecution(place, "Post Place Pose", true);
}
} // namespace iwtros2

// RCLCPP_COMPONENTS_REGISTER_NODE(iwtros2::IiwaMove)
