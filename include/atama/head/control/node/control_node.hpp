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

#ifndef ATAMA__HEAD__CONTROL__NODE__CONTROL_NODE_HPP_
#define ATAMA__HEAD__CONTROL__NODE__CONTROL_NODE_HPP_

#include <memory>
#include <string>

#include "atama_interfaces/msg/status.hpp"
#include "atama/head/process/head.hpp"
#include "rclcpp/rclcpp.hpp"

namespace atama::control
{

class ControlNode
{
public:
  using RunHead = atama_interfaces::msg::RunHead;
  using Status = atama_interfaces::msg::Status;

  explicit ControlNode(
    rclcpp::Node::SharedPtr node, std::shared_ptr<Head> head);

  void update();

private:
  std::string get_node_prefix() const;

  void run_head_callback(const RunHead::SharedPtr message);
  bool is_object_name_not_in_detection_result();
  bool check_move_by_angle();

  rclcpp::Node::SharedPtr node;
  std::shared_ptr<Head> head;

  rclcpp::Subscription<RunHead>::SharedPtr run_head_subscriber;
  rclcpp::Publisher<Status>::SharedPtr status_publisher;

  std::function<bool()> process;
};

}  // namespace atama::control

#endif  // ATAMA__HEAD__CONTROL__NODE__CONTROL_NODE_HPP_
