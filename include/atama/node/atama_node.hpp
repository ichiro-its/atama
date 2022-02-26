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

#ifndef ATAMA__NODE__ATAMA_NODE_HPP_
#define ATAMA__NODE__ATAMA_NODE_HPP_

#include "atama/head/head.hpp"
#include "atama/receiver/node/receiver_node.hpp"
#include "atama/sender/node/sender_node.hpp"
#include "atama_interfaces/action/run_head.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace atama
{

using atama_interfaces::action::RunHead;
using GoalHandleRunHead = rclcpp_action::ServerGoalHandle<RunHead>;

class AtamaNode
{
public:
  explicit AtamaNode(rclcpp::Node::SharedPtr node);

  void set_receiver_and_sender_node(std::shared_ptr<atama::head::Head> head);

private:
  std::shared_ptr<atama::head::Head> head;

  rclcpp::Node::SharedPtr node;
  rclcpp::TimerBase::SharedPtr node_timer;

  std::shared_ptr<atama::receiver::ReceiverNode> receiver_node;
  std::shared_ptr<atama::sender::SenderNode> sender_node;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const RunHead::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleRunHead> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleRunHead> goal_handle);

  rclcpp_action::Server<RunHead>::SharedPtr run_head_server;
};

}  // namespace atama

#endif  // ATAMA__NODE__ATAMA_NODE_HPP_
