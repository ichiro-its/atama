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

#ifndef ATAMA__RECEIVER__NODE__RECEIVER_NODE_HPP_
#define ATAMA__RECEIVER__NODE__RECEIVER_NODE_HPP_

#include <memory>
#include <string>

#include "atama/head/head.hpp"
#include "kansei_interfaces/msg/axis.hpp"
#include "ninshiki_interfaces/msg/detected_object.hpp"
#include "ninshiki_interfaces/msg/detected_objects.hpp"
#include "rclcpp/rclcpp.hpp"
#include "shisen_interfaces/msg/camera_config.hpp"
#include "tachimawari_interfaces/msg/current_joints.hpp"

namespace atama
{
namespace receiver
{

class ReceiverNode
{
public:
  ReceiverNode(
    const rclcpp::Node::SharedPtr & node = nullptr,
    const std::shared_ptr<atama::head::Head> & head = nullptr);

private:
  using Axis = kansei_interfaces::msg::Axis;
  using CameraConfig = shisen_interfaces::msg::CameraConfig;
  using DetectedObjects = ninshiki_interfaces::msg::DetectedObjects;
  using CurrentJoints = tachimawari_interfaces::msg::CurrentJoints;

  rclcpp::Node::SharedPtr node;
  rclcpp::TimerBase::SharedPtr node_timer;

  std::shared_ptr<head::Head> head;

  rclcpp::Subscription<CurrentJoints>::SharedPtr current_joints_subscriber;
  rclcpp::Subscription<Axis>::SharedPtr get_orientation_subsciber;
  rclcpp::Subscription<DetectedObjects>::SharedPtr get_detection_result_subsciber;
  rclcpp::Subscription<CameraConfig>::SharedPtr get_camera_config_subsciber;
  // TODO(nathan): minus subscriber for aruku to get position robot
};

}  // namespace receiver
}  // namespace atama

#endif  // ATAMA__RECEIVER__NODE__RECEIVER_NODE_HPP_
