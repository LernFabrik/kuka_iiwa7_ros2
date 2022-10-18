#ifndef IIWA_MANIPULATION_HPP
#define IIWA_MANIPULATION_HPP

#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <thread>
#include <chrono>
#include <functional>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "std_msgs/msg/bool.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "moveit/planning_interface/planning_interface.h"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit/planning_scene_interface/planning_scene_interface.h"
#include "moveit/robot_model/robot_model.h"
#include "moveit/robot_model/robot_model.h"
#include "moveit/robot_state/robot_state.h"
#include "moveit_visual_tools/moveit_visual_tools.h"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_listener.h"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "iwtros2_launch/gripper_controller.hpp"
#include "control_msgs/action/gripper_command.hpp"

namespace rvt = rviz_visual_tools;

namespace iwtros2
{
    class IiwaMove
    {
        public:
            explicit IiwaMove(const rclcpp::Node::SharedPtr& node);

            // todo plcCallback

            geometry_msgs::msg::PoseStamped generatePose(const double x, const double y, const double z, const double roll, const double pitch, const double yaw, const std::string baselink);
            
            void go_home(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> & group, const bool tmp_pose);
            /** Motion execution pipe line */
            void motionExecution(geometry_msgs::msg::PoseStamped pose, const std::string task, const bool linear);

            void motionContraints(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> & group);
            
            /** Pick and Place Pipeline */
            void pnpPipeLine(geometry_msgs::msg::PoseStamped pick,
                            geometry_msgs::msg::PoseStamped place,
                            const double offset, const bool tmp_pose);

            /** Rviz visual marker*/
            void visualMarkers(const geometry_msgs::msg::PoseStamped target_pose,
                                const moveit::planning_interface::MoveGroupInterface::Plan plan, const std::string task);
            
            /**
             * @brief Gripper Controller
             * 
             * @param action set "OPEN" or "CLOSE"
             */
            void gripper_control(const char * action);
            /** Main Execution */
            void run();
            void _ctrl_loop();
            void _tf_listner_loop();

        private:
            // todo sub and pub
            rclcpp::Node::SharedPtr _node;
            rclcpp::TimerBase::SharedPtr _ctrl_timer;
            rclcpp::TimerBase::SharedPtr _tf_timer;

            std::shared_ptr<GripperController> _gripper_controller;
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr _sub_gripper_feedback;

            std::shared_ptr<moveit::planning_interface::MoveGroupInterface> _group;
            std::shared_ptr<moveit_visual_tools::MoveItVisualTools> _visual_tools;
            Eigen::Isometry3d _text_pose;
            double _velocity_scalling, acceleration_scalling;
            bool ready_pick_pose;
            bool _gripper_succeeded;

            std::string _planner_pipeline, _planner_id, _reference_frame,  _ee_frame;

            void updatePlannerConfig(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> & group, const std::string plannerId, const double vel_scalling, const double acc_scalling);
            void gripper_status_callback(const std_msgs::msg::Bool::SharedPtr result);
    };
    
} // namespace iwtros2



#endif //IIWA_MANIPULATION_HPP