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

namespace atama
{
namespace sender
{

SenderNode::SenderNode(
  const rclcpp::Node::SharedPtr & node,
  const std::shared_ptr<Head> & head)
: node(node), head(head)
{
  if (node != NULL) {
    set_joints_publisher = node->create_publisher<tachimawari_interfaces::msg::SetJoints>(
      get_node_prefix() + "/set_joints", 10);
    set_head_publisher = node->create_publisher<atama_interfaces::msg::Head>(
      get_node_prefix() + "/set_head_data", 10);

    {
      using atama_interfaces::srv::RunHead;
      run_head_service = node->create_service<RunHead>(
        get_node_prefix() + "/run_head",
        [this](std::shared_ptr<RunHead::Request> request,
        std::shared_ptr<RunHead::Response> response) {
          rclcpp::Rate rcl_rate(8ms);

          // Assume the function is not exist
          bool is_function_exist = false;

          // Check existence of the function
          switch (request->function_id) {
            case head::Head::SCAN_CUSTOM:
              {
                this->head->set_scan_limit(
                  request->scan_param.left_limit,
                  request->scan_param.right_limit,
                  request->scan_param.top_limit,
                  request->scan_param.bottom_limit
                );
                is_function_exist = true;
                break;
              }
            case head::Head::TRACK_OBJECT:
              {
                this->head->object_name = request->track_param.object_name;
                is_function_exist = true;
                break;
              }
            case head::Head::MOVE_BY_ANGLE:
              {
                this->head->pan_angle_goal = request->move_by_angle_param.pan_angle;
                this->head->tilt_angle_goal = request->move_by_angle_param.tilt_angle;
                is_function_exist = true;
                break;
              }
            case head::Head::LOOK_TO_POSITION:
              {
                this->head->goal_position_x = request->look_to_param.goal_position_x;
                this->head->goal_position_y = request->look_to_param.goal_position_y;
                is_function_exist = true;
                break;
              }
            case head::Head::SCAN_UP:
            case head::Head::SCAN_DOWN:
            case head::Head::SCAN_VERTICAL:
            case head::Head::SCAN_HORIZONTAL:
            case head::Head::SCAN_MARATHON:
              {
                is_function_exist = true;
                break;
              }
          }

          // Ensure data joints is already gotten by receiver_node
          if (!this->head->joints.empty() && is_function_exist) {
            while (rclcpp::ok()) {
              rcl_rate.sleep();

              if (check_process_is_finished()) {
                break;
              } else {
                process(request->function_id);
              }
            }
          }

          if (rclcpp::ok()) {
            response->done_processing = true;
          } else {
            response->done_processing = false;
          }
        }
      );
    }
  }
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

}  // namespace sender
}  // namespace atama
