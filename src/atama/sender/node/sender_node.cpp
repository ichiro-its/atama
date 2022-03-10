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

#include <bits/stdc++.h>

#include <memory>
#include <set>
#include <string>

#include "atama/sender/node/sender_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tachimawari_interfaces/msg/set_joints.hpp"

using namespace std::chrono_literals;

namespace atama::sender
{

SenderNode::SenderNode(rclcpp::Node::SharedPtr node, std::shared_ptr<atama::head::Head> head)
: node(node), head(head)
{
  set_joints_publisher = node->create_publisher<tachimawari_interfaces::msg::SetJoints>(
    "/joint/set_joints", 10);
  set_head_publisher = node->create_publisher<atama_interfaces::msg::Head>(
    "/head_data/set_head_data", 10);
}

void SenderNode::publish_joints()
{
  auto joints_msg = tachimawari_interfaces::msg::SetJoints();

  const auto & joints = head->get_joints();
  auto & joint_msgs = joints_msg.joints;

  joint_msgs.resize(joints.size());
  for (size_t i = 0; i < joints.size() && i < joint_msgs.size(); ++i) {
    joint_msgs[i].id = joints[i].get_id();
    joint_msgs[i].position = joints[i].get_position();
  }

  set_joints_publisher->publish(joints_msg);
}

void SenderNode::publish_head_data()
{
  auto pan_tilt_msg = atama_interfaces::msg::Head();

  pan_tilt_msg.pan_angle = head->get_pan_angle();
  pan_tilt_msg.tilt_angle = head->get_tilt_angle();
  pan_tilt_msg.distance = head->calculate_distance_from_pan_tilt(
    head->get_pan_angle(), head->get_tilt_angle());

  set_head_publisher->publish(pan_tilt_msg);
}

void SenderNode::process(int function_id)
{
  using atama::head::Head;

  if (!head->is_joint_empty()) {
    switch (function_id) {
      case Head::SCAN_UP: head->scan_up(); break;
      case Head::SCAN_DOWN: head->scan_down(); break;
      case Head::SCAN_VERTICAL: head->scan_vertical(); break;
      case Head::SCAN_HORIZONTAL: head->scan_horizontal(); break;
      case Head::SCAN_MARATHON: head->scan_marathon(); break;
      case Head::SCAN_CUSTOM: head->scan_custom(Head::SCAN_CUSTOM); break;
      case Head::TRACK_OBJECT:
        head->track_object(head->object_name);
        break;
      case Head::MOVE_BY_ANGLE:
        head->move_by_angle(
          head->pan_angle_goal,
          head->tilt_angle_goal);
        break;
      // TODO(nathan): implement look_to_position() after
      // robot_position_x, robot_position_y, yaw are obtained
      case Head::LOOK_TO_POSITION:
        break;
    }
  }

  publish_joints();
  publish_head_data();
}

bool SenderNode::is_detection_result_empty()
{
  std::set<std::string> result_name;
  for (const auto & name : head->detection_result) {
    result_name.insert(name.label);
  }

  // Object not found
  return result_name.find(head->object_name) == result_name.end();
}

bool SenderNode::check_move_by_angle()
{
  return head->get_pan_angle() == head->pan_angle_goal &&
         head->get_tilt_angle() == head->tilt_angle_goal;
}

bool SenderNode::check_process_is_finished()
{
  using atama::head::Head;

  switch (head->function_id) {
    case Head::SCAN_UP:
    case Head::SCAN_DOWN:
    case Head::SCAN_VERTICAL:
    case Head::SCAN_HORIZONTAL:
    case Head::SCAN_MARATHON:
    case Head::SCAN_CUSTOM: return !is_detection_result_empty(); break;
    case Head::TRACK_OBJECT: return is_detection_result_empty(); break;
    case Head::MOVE_BY_ANGLE: return check_move_by_angle(); break;
    // TODO(nathan): implement look_to_position() after
    // robot_position_x, robot_position_y, yaw are obtained
    case Head::LOOK_TO_POSITION: break;
  }
}

std::string SenderNode::get_node_prefix()
{
  return "sender";
}

}  // namespace atama::sender
