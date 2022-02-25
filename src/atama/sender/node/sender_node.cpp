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

#include <memory>
#include <string>
#include <vector>

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
  set_pan_tilt_publisher = node->create_publisher<atama_interfaces::msg::Head>(
    "/pan_tilt/set_pan_tilt", 10);
  function_id = -1;
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

void SenderNode::publish_pan_tilt()
{
  auto pan_tilt_msg = atama_interfaces::msg::Head();

  pan_tilt_msg.pan_angle = head->get_pan_angle();
  pan_tilt_msg.tilt_angle = head->get_tilt_angle();
  pan_tilt_msg.distance = head->calculate_distance_from_pan_tilt(
    head->get_pan_angle(), head->get_tilt_angle());

  set_pan_tilt_publisher->publish(pan_tilt_msg);
}

bool SenderNode::is_function_exist(std::string function_name)
{
  try {
    function_id = head->map.at(function_name);
    return true;
  }
  catch(...) {
    return false;
  }
}

void SenderNode::process()
{
  if (!head->is_joint_empty())
  {
    switch (function_id) {
      case atama::head::Head::SCAN_UP: {head->scan_up(); break;}
      case atama::head::Head::SCAN_DOWN: {head->scan_down(); break;}              
      // case head->SCAN_VERTICAL: {head->scan_vertical(); break;}          
      // case head->SCAN_HORIZONTAL: {head->scan_horizontal(); break;}        
      // case head->SCAN_MARATHON: {head->scan_marathon(); break;}          
      // case head->SCAN_CUSTOM: {head->scan_custom(); break;}            
      // case head->TRACKING: {head->tracking(); break;}               
      // case head->TRACKING_PAN_ONLY: {head->tracking_pan_only(); break;}      
      // case head->TRACKING_TILT_ONLY: {head->tracking_tilt_only(); break;}     
      // case head->MOVE_BY_ANGLE: {head->move_by_angle(); break;}
      // for robot_position_x, robot_position_y, yaw get from receiver_node          
      // case head->LOOK_TO_POSITION: {head->look_to_position(); break;}       
    }
  }

  publish_pan_tilt();
}

std::string SenderNode::get_node_prefix() const
{
  return "sender";
}

}  // namespace atama::sender
