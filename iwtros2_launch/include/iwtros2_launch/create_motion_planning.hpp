#ifndef CREATE_MOTION_PLANNING_HPP
#define CREATE_MOTION_PLANNING_HPP

#include <assert.h>
#include <chrono>
#include <functional>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/bool.hpp"

#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_interface/planning_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit/robot_model/robot_model.h"
#include "moveit/robot_state/robot_state.h"
#include "moveit_visual_tools/moveit_visual_tools.h"
#include <moveit/planning_pipeline/planning_pipeline.h>

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_listener.h"

#include "iwtros2_launch/arm_utilities.hpp"

namespace rvt = rviz_visual_tools;

namespace iwtros2
{
/**
 * @brief KUKA IIWA Motiontion controller using Movegrou Interface
 */
class CreateMotion
{
  public:
    robot_config conf;

    explicit CreateMotion(const rclcpp::Node::SharedPtr &node,
                          const std::shared_ptr<moveit::planning_interface::MoveGroupInterface> &group);

    bool joint_space_goal(const std::vector<double> &joint_values, const moveit::core::RobotStatePtr robot_state,
                          moveit::planning_interface::MoveGroupInterface::Plan &plan);

    bool pose_goal(const geometry_msgs::msg::PoseStamped &pose, const moveit::core::RobotStatePtr robot_state,
                   moveit::planning_interface::MoveGroupInterface::Plan &plan, const bool is_linear = false);

    /** Rviz visual marker*/
    void visualMarkers(const geometry_msgs::msg::PoseStamped target_pose,
                       const moveit::planning_interface::MoveGroupInterface::Plan plan, const std::string task);

  private:
    rclcpp::Node::SharedPtr _node;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> _group;

    robot_model_loader::RobotModelLoaderPtr _robot_model_loader;
    planning_scene_monitor::PlanningSceneMonitorPtr _psm;
    moveit::core::RobotModelPtr _robot_model;
    moveit::core::RobotStatePtr _robot_state;
    const moveit::core::JointModelGroup *_joint_model_group;
    std::shared_ptr<planning_pipeline::PlanningPipeline> _planning_pipeline;

    std::shared_ptr<moveit_visual_tools::MoveItVisualTools> _visual_tools;
    Eigen::Isometry3d _text_pose;
};

} // namespace iwtros2

#endif // CREATE_MOTION_PLANNING_HPP
