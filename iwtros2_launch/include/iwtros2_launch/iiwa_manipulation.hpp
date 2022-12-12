#ifndef IIWA_MANIPULATION_HPP
#define IIWA_MANIPULATION_HPP

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
#include <moveit/planning_pipeline/planning_pipeline.h>
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit/robot_model/robot_model.h"
#include "moveit/robot_state/robot_state.h"
#include "moveit_visual_tools/moveit_visual_tools.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_listener.h"

#include "control_msgs/action/gripper_command.hpp"
#include "iwtros2_launch/gripper_controller.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "iwtros2_interface/msg/kuka_control.hpp"
#include "iwtros2_interface/msg/plc_control.hpp"

namespace rvt = rviz_visual_tools;

namespace iwtros2
{
/**
 * @brief KUKA IIWA Motiontion controller using Movegrou Interface
 */
class IiwaMove
{
  public:
    explicit IiwaMove(const rclcpp::Node::SharedPtr &node,
                      const std::shared_ptr<moveit::planning_interface::MoveGroupInterface> &group);

    // todo plcCallback
    /**
     * @brief Generate ROS2 message type
     *
     * @param x
     * @param y
     * @param z
     * @param roll
     * @param pitch
     * @param yaw
     * @param baselink Robot base link name
     * @return geometry_msgs::msg::PoseStamped ROS2 message
     */
    geometry_msgs::msg::PoseStamped generatePose(const double x, const double y, const double z, const double roll,
                                                 const double pitch, const double yaw, const std::string baselink);

    /**
     * @brief Joint Space Action to go default home position
     * @param tmp_pose False for default home position and True for Temporary position in where product is loaded
     */
    void go_home(const bool tmp_pose);
    /**
     * @brief Motion execution pipe line
     *
     * @param pose Robot Pose
     * @param task String task name
     * @param linear True for linear motion or False for Point to Point motion
     */
    void motionExecution(geometry_msgs::msg::PoseStamped pose, const std::string task, const bool linear);
    /**
     * @brief Define motion constrains
     * @warning Do not use this (todo: Test)
     * @param group
     */
    void motionContraints(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> &group);

    /**
     * @brief Pick and Place Pipeline
     *
     * @param pick
     * @param place
     * @param offset Define distance where pick or place motion begins
     * @param tmp_pose
     */
    void pnpPipeLine(geometry_msgs::msg::PoseStamped pick, geometry_msgs::msg::PoseStamped place, const double offset,
                     const bool tmp_pose);

    /** Rviz visual marker*/
    void visualMarkers(const geometry_msgs::msg::PoseStamped target_pose,
                       const moveit::planning_interface::MoveGroupInterface::Plan plan, const std::string task);

  private:
    rclcpp::Node::SharedPtr _node;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> _group;

    std::shared_ptr<GripperController> _gripper_client;

    robot_model_loader::RobotModelLoaderPtr _robot_model_loader;
    planning_scene_monitor::PlanningSceneMonitorPtr _psm;
    moveit::core::RobotModelPtr _robot_model;
    moveit::core::RobotStatePtr _robot_state;
    const moveit::core::JointModelGroup * _joint_model_group;
    std::shared_ptr<planning_pipeline::PlanningPipeline> _planning_pipeline;

    std::shared_ptr<moveit_visual_tools::MoveItVisualTools> _visual_tools;
    Eigen::Isometry3d _text_pose;
    double _velocity_scalling, acceleration_scalling;
    bool ready_pick_pose;
    bool _gripper_succeeded;

    std::string _planner_pipeline, _planner_id, _reference_frame, _ee_frame;

    void updatePlannerConfig(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> &group,
                             const std::string plannerId, const double vel_scalling, const double acc_scalling);
    void gripper_status_callback(const std_msgs::msg::Bool::SharedPtr result);
};

/**
 * @brief Manage PLC Control
 */
class ControlPLC
{
  public:
    bool move_home, conveyor_pick, hochregallager_pick;
    explicit ControlPLC(const rclcpp::Node::SharedPtr &node) : _node(node)
    {
        using namespace std::placeholders;
        _pub = _node->create_publisher<iwtros2_interface::msg::PlcControl>("plc_control", 10);
        _sub = _node->create_subscription<iwtros2_interface::msg::KukaControl>(
            "kuka_control", 10, std::bind(&ControlPLC::callback, this, _1));
        this->move_home = false;
        this->conveyor_pick = false;
        this->hochregallager_pick = false;
    }
    /**
     * @brief Publish message for PLC write
     *
     * @param reached_home
     * @param conveyor_placed
     * @param hochregallager_placed
     */
    void plc_publish(const bool reached_home = false, const bool conveyor_placed = false,
                     const bool hochregallager_placed = false)
    {
        iwtros2_interface::msg::PlcControl msg;
        msg.reached_home = reached_home;
        msg.conveyor_placed = conveyor_placed;
        msg.hochregallager_placed = hochregallager_placed;
        this->_pub->publish(msg);
    }

  private:
    rclcpp::Node::SharedPtr _node;
    rclcpp::Publisher<iwtros2_interface::msg::PlcControl>::SharedPtr _pub;
    rclcpp::Subscription<iwtros2_interface::msg::KukaControl>::SharedPtr _sub;

    void callback(const iwtros2_interface::msg::KukaControl::SharedPtr msg)
    {
        this->move_home = msg->move_home;
        this->conveyor_pick = msg->conveyor_pick;
        this->hochregallager_pick = msg->hochregallager_pick;
    }
};

} // namespace iwtros2

#endif // IIWA_MANIPULATION_HPP