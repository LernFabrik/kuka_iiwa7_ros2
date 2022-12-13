#ifndef GRIPPER_CONTROLLER_HPP
#define GRIPPER_CONTROLLER_HPP

#include "control_msgs/action/gripper_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include <assert.h>
#include <chrono>
#include <functional>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <thread>

using namespace std::placeholders;

namespace iwtros2
{
class GripperController
{
  public:
    using GripperCommand = control_msgs::action::GripperCommand;
    using ClientGoalHandle = rclcpp_action::ClientGoalHandle<GripperCommand>;

    explicit GripperController(const rclcpp::Node::SharedPtr &node) : _node(node)
    {
        RCLCPP_INFO(_node->get_logger(), "Creating Gripper Client");
        this->_client = rclcpp_action::create_client<GripperCommand>(_node, "/wsg50_gripper_driver/gripper_action");
    }

    std::shared_future<std::shared_ptr<ClientGoalHandle>> open()
    {
        if (!this->_client->wait_for_action_server())
        {
            RCLCPP_ERROR(_node->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        }
        auto options = rclcpp_action::Client<GripperCommand>::SendGoalOptions();
        options.goal_response_callback = std::bind(&GripperController::gripper_goal_response_callback, this, _1);
        options.result_callback = std::bind(&GripperController::gripper_result_callback, this, _1);

        auto open_goal = GripperCommand::Goal();

        RCLCPP_INFO(_node->get_logger(), "Sending Gripper OPEN Command .. ");
        open_goal.command.position = 0.054;
        open_goal.command.max_effort = 0.0;
        auto gh_future = _client->async_send_goal(open_goal, options);
        return gh_future;
    }

    std::shared_future<std::shared_ptr<ClientGoalHandle>> close()
    {
        if (!this->_client->wait_for_action_server())
        {
            RCLCPP_ERROR(_node->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
        }
        auto options = rclcpp_action::Client<GripperCommand>::SendGoalOptions();
        options.goal_response_callback = std::bind(&GripperController::gripper_goal_response_callback, this, _1);
        options.result_callback = std::bind(&GripperController::gripper_result_callback, this, _1);

        auto close_goal = GripperCommand::Goal();

        RCLCPP_INFO(_node->get_logger(), "Sending Gripper CLOSE Command .. ");
        close_goal.command.position = 0.005;
        close_goal.command.max_effort = 10.0;
        auto gh_future = _client->async_send_goal(close_goal, options);
        return gh_future;
    }

  private:
    rclcpp::Node::SharedPtr _node;
    rclcpp_action::Client<GripperCommand>::SharedPtr _client;

    void gripper_goal_response_callback(const ClientGoalHandle::SharedPtr &goal_handle)
    {
        if (!goal_handle)
        {
            RCLCPP_ERROR(_node->get_logger(), "Gripper Goal was rejected by the server");
        }
        else
        {
            RCLCPP_INFO(_node->get_logger(), "Gripper Goal accepted by server, waiting");
        }
    }

    void gripper_result_callback(const ClientGoalHandle::WrappedResult &result)
    {
        switch (result.code)
        {
        case rclcpp_action::ResultCode::SUCCEEDED:
            RCLCPP_INFO(_node->get_logger(), "Gripper Goal was success");
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(_node->get_logger(), "Gripper Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(_node->get_logger(), "Gripper Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(_node->get_logger(), "Gripper Unknown result code");
            return;
        }
    }
};

} // namespace iwtros2

#endif // IIWA_MANIPULATION_HPP
