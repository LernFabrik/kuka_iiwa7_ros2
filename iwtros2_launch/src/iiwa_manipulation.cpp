#include "iwtros2_launch/iiwa_manipulation.hpp"

using namespace std::placeholders;

namespace iwtros2
{
    /**
     * @brief Construct a new Iiwa Move:: Iiwa Move object
     * 
     * @param options ros2 options
     */
    IiwaMove::IiwaMove(const rclcpp::NodeOptions &options) : Node("~/iiwa_motion_controller_node", options)
    {
        // todo initiallize more variables
        // Initialize sub and pub
        // Initialize parameretes
        // Setup Move group planner
        this->_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>("iiwa_arm");
        _group->setPlanningPipelineId("pilz_industrial_motion_planner");
        _group->setPlannerId("PTP");
        _group->setMaxVelocityScalingFactor(0.2);
        _group->setMaxAccelerationScalingFactor(0.3);
        _group->setPoseReferenceFrame("iiwa_link_0");
        _group->setEndEffector("iiwa_tool0");
        _group->allowReplanning(true);

        this->_visual_tools = std::make_shared<moveit_visual_tools::MoveItVisualTools>(this, "iiwa_link_0", "trajectory_marker", _group->getRobotModel());


        // Todo Load Positon parameters from yaml file.
        this->_ctrl_loop = this->create_wall_timer(rclcpp::WallRate(1).period(), std::bind(&IiwaMove::_ctrl_loop, this));
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

    void IiwaMove::visualMarkers(const geometry_msgs::msg::PoseStamped & target_pose,
                                moveit::planning_interface::MoveGroupInterface::Plan plan, const std::string task)
    {
        _visual_tools->deleteAllMarkers();
        _visual_tools->loadRemoteControl();

        Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
        text_pose.translation().z() = 1.0;
        _visual_tools->publishText(text_pose, "MoveGroupInterface_Demo", rvt::WHITE, rvt::XLARGE);

        
    }

    void IiwaMove::motionExecution(const geometry_msgs::msg::PoseStamped pose)
    {
        _group->setPoseTarget(pose);

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        moveit::core::MoveItErrorCode eCode = _group->plan(plan);

        RCLCPP_INFO(this->get_logger(), "Motion Planning is: %s", eCode?"Success":"Failed");

        if(eCode) _group->execute(plan);
        else RCLCPP_ERROR(this->get_logger(), "Failed to generate motion plan"); // Todo: Do something.
        //_group->move();
    }

    void IiwaMove::pnpPipeLine(geometry_msgs::msg::PoseStamped pick,
                            geometry_msgs::msg::PoseStamped place,
                            const double offset)
    {
        pick.pose.position.z += offset;
        motionExecution(pick);
        // todo: Open gripper
        pick.pose.position.z -=offset;
        motionExecution(pick);
        // Close Gripper
        //rclcpp::sleep_for(1.0);
        pick.pose.position.z += offset;
        motionExecution(pick);
        // Place
        place.pose.position.z += offset;
        motionExecution(place);
        place.pose.position.z -= offset;
        motionExecution(place);
        // Open Gripper
        // Sleep
    }

    void IiwaMove::_ctrl_loop()
    {
        
    }
} // namespace iwtros2
