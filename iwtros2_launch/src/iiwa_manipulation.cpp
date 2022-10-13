#include "iwtros2_launch/iiwa_manipulation.hpp"

#include "moveit_msgs/msg/orientation_constraint.hpp"
#include "moveit_msgs/msg/constraints.hpp"

using namespace std::placeholders;

namespace iwtros2
{
    /**
     * @brief Construct a new Iiwa Move:: Iiwa Move object
     * 
     * @param options ros2 options
     */
    IiwaMove::IiwaMove(const rclcpp::Node::SharedPtr& node): _node(node)
    {
        RCLCPP_INFO(_node->get_logger(), "Move group node is starting");
        // todo initiallize more variables
        // Initialize sub and pub
        // Initialize parameretes

        // Setup Move group planner
        this->_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(_node, "iiwa_arm");
        _group->setPlanningPipelineId("pilz");
        _group->setPlannerId("PTP");
        _group->setMaxVelocityScalingFactor(0.2);
        _group->setMaxAccelerationScalingFactor(0.1);
        _group->setPoseReferenceFrame("iiwa7_link_0");
        _group->setEndEffector("iiwa7_tool0");
        _group->allowReplanning(true);

        // Display basic information
        RCLCPP_INFO(_node->get_logger(), "Planning frame: %s", _group->getPlanningFrame().c_str());
        RCLCPP_INFO(_node->get_logger(), "End effector link: %s", _group->getEndEffector().c_str());

        this->_visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools>(_node, "iiwa_link_0", "trajectory_marker", _group->getRobotModel());
        _visual_tools->deleteAllMarkers();
        //_visual_tools->loadRemoteControl();

        this->_text_pose = Eigen::Isometry3d::Identity();
        _text_pose.translation().z() = 1.0;
        _visual_tools->publishText(_text_pose, "MoveGroupInterface_IIWA7", rvt::WHITE, rvt::XLARGE);
        _visual_tools->trigger();


        // Todo Load Positon parameters from yaml file.
        this->_ctrl_timer = _node->create_wall_timer(rclcpp::WallRate(1).period(), std::bind(&IiwaMove::_ctrl_loop, this));
    }

    geometry_msgs::msg::PoseStamped IiwaMove::generatePose(const double x, const double y, const double z, const double roll, const double pitch, const double yaw, const std::string baselink)
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

    void IiwaMove::go_home(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> & group, const bool tmp_pose)
    {
        // const moveit::core::JointModelGroup * joint_model_group = _group->getCurrentState()->getJointModelGroup("iiwa_arm");
        // moveit::core::RobotStatePtr current_state = group->getCurrentState(10);

        std::vector<double> joint_group_position;
        // current_state->copyJointGroupPositions(joint_model_group, joint_group_position);
        if (tmp_pose) 
        {
            joint_group_position.push_back(1.5708);
            joint_group_position.push_back(-0.26);
            joint_group_position.push_back(0.0);
            joint_group_position.push_back(-1.74533);
            joint_group_position.push_back(0.0);
            joint_group_position.push_back(1.74533);
            joint_group_position.push_back(0.0);
        }
        else 
        {
            joint_group_position.push_back(0.0);
            joint_group_position.push_back(0.0);
            joint_group_position.push_back(0.0);
            joint_group_position.push_back(-1.5708);
            joint_group_position.push_back(0.0);
            joint_group_position.push_back(1.5708);
            joint_group_position.push_back(0.0);
        }
        group->setJointValueTarget(joint_group_position);

        group->setMaxVelocityScalingFactor(0.2);
        group->setMaxAccelerationScalingFactor(0.3);

        moveit::planning_interface::MoveGroupInterface::Plan my_plan;

        bool success = (group->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);
        RCLCPP_INFO(_node->get_logger(), "Visualizing plan 2 (joint space goal) %s", success ? "" : "FAILED");
        group->execute(my_plan);
    }

    void IiwaMove::motionContraints(std::shared_ptr<moveit::planning_interface::MoveGroupInterface> & group)
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
        const moveit::core::JointModelGroup * joint_model_group = _group->getCurrentState()->getJointModelGroup("iiwa_arm");
        _visual_tools->publishAxisLabeled(target_pose.pose, task.c_str());
        _visual_tools->publishText(_text_pose, task.c_str(), rvt::WHITE, rvt::XLARGE);
        _visual_tools->publishTrajectoryLine(plan.trajectory_, joint_model_group);
        _visual_tools->trigger();
    }

    void IiwaMove::motionExecution(geometry_msgs::msg::PoseStamped pose, const std::string task, const bool linear)
    {
        _group->setMaxVelocityScalingFactor(0.2);
        _group->setMaxAccelerationScalingFactor(0.3);

        if(linear)
        {
            _group->setPlannerId("LIN");
            _group->setMaxVelocityScalingFactor(0.1);
            pose.header.frame_id = "world";
        } else {
            _group->setPlannerId("PTP");
        }

        _group->setPoseTarget(pose);

        //motionContraints(_group);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        moveit::core::MoveItErrorCode eCode = _group->plan(plan);

        RCLCPP_INFO(_node->get_logger(), "Motion Planning is: %s", eCode?"Success":"Failed");

        //visualMarkers(pose, plan, task);

        if(eCode) _group->execute(plan);
        else RCLCPP_ERROR(_node->get_logger(), "Failed to generate motion plan"); // Todo: Do something.
        //_group->move();
    }

    void IiwaMove::pnpPipeLine(geometry_msgs::msg::PoseStamped pick,
                            geometry_msgs::msg::PoseStamped place,
                            const double offset, const bool tmp_pose)
    {
        pick.pose.position.z += offset;
        motionExecution(pick, "Pre-Pick Pose", false);
        // todo: Open gripper
        pick.pose.position.z -=offset;
        motionExecution(pick, "Pick Pose", true);
        // Close Gripper
        pick.pose.position.z += offset;
        motionExecution(pick, "Post Pick Pose", true);
        
        if (tmp_pose) 
        {
            // geometry_msgs::msg::PoseStamped inter_pose = generatePose(-0.0, 0.3, 1.5, M_PI, 0, 3 * M_PI/4, "iiwa7_link_0");
            // motionExecution(place, "Intermidiate pose");
            go_home(_group, true);
        }
        // Place
        place.pose.position.z += offset;
        motionExecution(place, "Pre Place Pose", false);
        place.pose.position.z -= offset;
        motionExecution(place, "Place Pose", true);
        place.pose.position.z += offset;
        motionExecution(place, "Post Place Pose", true);
        // Open Gripper
    }

    void IiwaMove::_ctrl_loop()
    {
        geometry_msgs::msg::PoseStamped home_pose = generatePose(0.5, 0, 1.61396, - M_PI, 0, M_PI, "iiwa7_link_0");
        geometry_msgs::msg::PoseStamped conveyor_pick_pose = generatePose(0.235, -0.43, 1.4, M_PI, 0, M_PI/4, "iiwa7_link_0"); // 1.221
        geometry_msgs::msg::PoseStamped hochregallager_place_pose = generatePose(0.5, 0, 1.4, M_PI, 0, 3 * M_PI/4, "iiwa7_link_0");
        geometry_msgs::msg::PoseStamped loading_place_pose = generatePose(0.0, 0.5, 1.2, M_PI, 0, 3 * M_PI/4, "iiwa7_link_0");

        // motionExecution(home_pose, "Go to Home");
        go_home(_group, false);
        rclcpp::sleep_for(std::chrono::milliseconds(500));
        pnpPipeLine(conveyor_pick_pose, hochregallager_place_pose, 0.15, false);
        go_home(_group, false);
        pnpPipeLine(hochregallager_place_pose, loading_place_pose, 0.15, true);
        go_home(_group, true);
        go_home(_group, false);
    }
} // namespace iwtros2


// RCLCPP_COMPONENTS_REGISTER_NODE(iwtros2::IiwaMove)