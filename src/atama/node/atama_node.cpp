// Copyright (c) 2021 Ichiro ITS
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.

#include <chrono>
#include <memory>
#include <string>

#include "atama/node/atama_node.hpp"
#include "atama_interfaces/action/run_head.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace atama
{

AtamaNode::AtamaNode(rclcpp::Node::SharedPtr node)
: node(node), receiver_node(nullptr), sender_node(nullptr),
  done_get_joints_data(false)
{
  run_head_server = rclcpp_action::create_server<RunHead>(
    node->get_node_base_interface(),
    node->get_node_clock_interface(),
    node->get_node_logging_interface(),
    node->get_node_waitables_interface(),
    "run_head",
    std::bind(&AtamaNode::handle_goal, this, _1, _2),
    std::bind(&AtamaNode::handle_cancel, this, _1),
    std::bind(&AtamaNode::handle_accepted, this, _1)
  );

  node_timer = node->create_wall_timer(
    8ms,
    [this]() {
      if (receiver_node != nullptr) {
        done_get_joints_data = receiver_node->get_joints_data();
      }
    }
  );
}

void AtamaNode::set_receiver_and_sender_node(std::shared_ptr<atama::head::Head> head)
{
  this->head = head;
  receiver_node = std::make_shared<atama::receiver::ReceiverNode>(
    node, head);
  sender_node = std::make_shared<atama::sender::SenderNode>(
    node, head);
}

rclcpp_action::GoalResponse AtamaNode::handle_goal(
  const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const RunHead::Goal> goal)
{
  bool is_function_exist = false;

  if (sender_node) {
    head->set_function_id(goal->function_id);

    switch (goal->function_id) {
      case atama::head::Head::SCAN_CUSTOM:
        {
          head->set_scan_limit(
            goal->scan_param.left_limit,
            goal->scan_param.right_limit,
            goal->scan_param.top_limit,
            goal->scan_param.bottom_limit
          );
          is_function_exist = true;
          break;
        }
      case atama::head::Head::TRACK_OBJECT:
        {
          head->set_object_name(goal->track_param.object_name);
          is_function_exist = true;
          break;
        }
      case atama::head::Head::MOVE_BY_ANGLE:
        {
          head->set_pan_angle_goal(goal->move_by_angle_param.pan_angle);
          head->set_tilt_angle_goal(goal->move_by_angle_param.tilt_angle);
          is_function_exist = true;
          break;
        }
      case atama::head::Head::LOOK_TO_POSITION:
        {
          head->set_goal_position_x(goal->look_to_param.goal_position_x);
          head->set_goal_position_y(goal->look_to_param.goal_position_y);
          is_function_exist = true;
          break;
        }
      case atama::head::Head::SCAN_UP:
      case atama::head::Head::SCAN_DOWN:
      case atama::head::Head::SCAN_VERTICAL:
      case atama::head::Head::SCAN_HORIZONTAL:
      case atama::head::Head::SCAN_MARATHON:
        {
          is_function_exist = true;
          break;
        }
    }
  }

  return is_function_exist ?
         rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE :
         rclcpp_action::GoalResponse::REJECT;
}

rclcpp_action::CancelResponse AtamaNode::handle_cancel(
  const std::shared_ptr<GoalHandleRunHead> goal_handle)
{
  return rclcpp_action::CancelResponse::ACCEPT;
}

void AtamaNode::handle_accepted(const std::shared_ptr<GoalHandleRunHead> goal_handle)
{
  std::thread{[this](const std::shared_ptr<GoalHandleRunHead> goal_handle) {
      rclcpp::Rate rcl_rate(8ms);

      const auto goal = goal_handle->get_goal();
      auto feedback = std::make_shared<RunHead::Feedback>();
      auto result = std::make_shared<RunHead::Result>();

      if (done_get_joints_data) {
        while (rclcpp::ok()) {
          rcl_rate.sleep();

          if (goal_handle->is_canceling()) {
            result->done_processing = false;
            goal_handle->canceled(result);
            return;
          }

          if (sender_node->check_process_is_finished()) {
            break;
          } else {
            sender_node->process(goal->function_id);
          }

          goal_handle->publish_feedback(feedback);
        }
      }

      if (rclcpp::ok()) {
        result->done_processing = true;
        goal_handle->succeed(result);
      }
    }, goal_handle}.join();
}

}  // namespace atama
