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

#ifndef ATAMA__HEAD__NODE__HEAD_NODE_HPP_
#define ATAMA__HEAD__NODE__HEAD_NODE_HPP_

#include <memory>
#include <string>

#include "atama/head/process/head.hpp"
#include "atama_interfaces/msg/head.hpp"
#include "atama_interfaces/srv/run_head.hpp"
#include "kansei_interfaces/msg/axis.hpp"
#include "ninshiki_interfaces/msg/detected_object.hpp"
#include "ninshiki_interfaces/msg/detected_objects.hpp"
#include "rclcpp/rclcpp.hpp"
#include "shisen_interfaces/msg/camera_config.hpp"
#include "tachimawari_interfaces/msg/current_joints.hpp"
#include "tachimawari_interfaces/msg/set_joints.hpp"

namespace atama
{

class HeadNode
{
public:
  HeadNode(
    rclcpp::Node::SharedPtr node = nullptr,
    std::shared_ptr<Head> head = nullptr);

  void publish_joints();
  // change function name
  void publish_head_data();

  void process(int function_id);
  bool check_process_is_finished();

private:
  using Axis = kansei_interfaces::msg::Axis;
  using CameraConfig = shisen_interfaces::msg::CameraConfig;
  using DetectedObjects = ninshiki_interfaces::msg::DetectedObjects;
  using CurrentJoints = tachimawari_interfaces::msg::CurrentJoints;

  rclcpp::Node::SharedPtr node;

  std::shared_ptr<Head> head;
  int req_function_id;

  rclcpp::Subscription<CurrentJoints>::SharedPtr current_joints_subscriber;
  rclcpp::Subscription<Axis>::SharedPtr get_orientation_subsciber;
  rclcpp::Subscription<DetectedObjects>::SharedPtr get_detection_result_subsciber;
  rclcpp::Subscription<CameraConfig>::SharedPtr get_camera_config_subsciber;
  // TODO(nathan): minus subscriber for aruku to get position robot

  rclcpp::Publisher<tachimawari_interfaces::msg::SetJoints>::SharedPtr set_joints_publisher;
  rclcpp::Publisher<atama_interfaces::msg::Head>::SharedPtr set_head_publisher;
  rclcpp::Service<atama_interfaces::srv::RunHead>::SharedPtr run_head_service;

  bool is_detection_result_empty();
  bool check_move_by_angle();

  static std::string get_node_prefix();
};

}  // namespace atama

#endif  // ATAMA__HEAD__NODE__HEAD_NODE_HPP_
