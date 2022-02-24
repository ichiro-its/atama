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

class SenderNode
{
public:
  SenderNode(
    rclcpp::Node::SharedPtr node,
    std::shared_ptr<atama::head::Head> head);
  
  void publish_joints();
  // change function name
  void publish_pan_tilt();

  bool is_function_exist(std::string function_name);
  void process();
private:
  rclcpp::Node::SharedPtr node;

  std::string get_node_prefix() const;

  std::shared_ptr<atama::head::Head> head;

  int function_id;

  rclcpp::Publisher<tachimawari_interfaces::msg::SetJoints>::SharedPtr set_joints_publisher;
  // change variable name
  rclcpp::Publisher<atama_interfaces::msg::Head>::SharedPtr set_pan_tilt_publisher;
};

}  // namespace atama::sender

#endif  // ATAMA__SENDER__NODE__SENDER_NODE_HPP_
