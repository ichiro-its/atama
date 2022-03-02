// Copyright (c) 2021 Ichiro ITS
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef ATAMA__SENDER__NODE__SENDER_NODE_HPP_
#define ATAMA__SENDER__NODE__SENDER_NODE_HPP_

#include <memory>
#include <string>

#include "atama/head/head.hpp"
#include "atama_interfaces/msg/head.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tachimawari_interfaces/msg/set_joints.hpp"

namespace atama::sender
{

struct FunctionParam
{
  double left_limit;
  double right_limit;
  double top_limit;
  double bottom_limit;
  int scan_type;
  std::string object_name;
  double pan_angle;
  double tilt_angle;
  double goal_position_x;
  double goal_position_y;
};

class SenderNode
{
public:
  SenderNode(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<atama::head::Head> head);

  void publish_joints();
  // change function name
  void publish_head_data();

  void process(int function_id);
  bool check_process_is_finished();

private:
  rclcpp::Node::SharedPtr node;

  std::shared_ptr<atama::head::Head> head;

  rclcpp::Publisher<tachimawari_interfaces::msg::SetJoints>::SharedPtr set_joints_publisher;
  rclcpp::Publisher<atama_interfaces::msg::Head>::SharedPtr set_head_publisher;

  bool check_scan();
  bool check_track();
  bool check_move_by_angle();

  std::string get_node_prefix() const;
};

}  // namespace atama::sender

#endif  // ATAMA__SENDER__NODE__SENDER_NODE_HPP_
