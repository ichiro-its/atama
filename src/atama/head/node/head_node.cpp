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
#include <set>
#include <string>
#include <vector>

#include "atama/head/node/head_node.hpp"
#include "kansei_interfaces/msg/axis.hpp"
#include "ninshiki_interfaces/msg/detected_object.hpp"
#include "ninshiki_interfaces/msg/detected_objects.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace atama
{

HeadNode::HeadNode(rclcpp::Node::SharedPtr node, std::shared_ptr<Head> head)
: node(node), head(head)
{
  using ninshiki_interfaces::msg::DetectedObject;

  if (node != NULL) {
    get_orientation_subsciber = node->create_subscription<Axis>(
      "measurement/orientation", 10,
      [this](const Axis::SharedPtr message) {
        // this->head->set_yaw(message->orientation[2]);
      }
    );

    get_detection_result_subsciber =
      node->create_subscription<DetectedObjects>(
      "ninshiki_cpp/detection", 10,
      [this](const DetectedObjects::SharedPtr message) {
        std::vector<DetectedObject> temp_detection_result;

        for (const auto & detected_object : message->detected_objects) {
          temp_detection_result.push_back(detected_object);
        }

        this->head->detection_result = temp_detection_result;
      }
      );

    get_camera_config_subsciber = node->create_subscription<CameraConfig>(
      "/camera/camera_config", 10,
      [this](const CameraConfig::SharedPtr message) {
        this->head->camera_width = message->width;
        this->head->camera_height = message->height;
        this->head->view_v_angle = message->v_angle;
        this->head->view_h_angle = message->h_angle;
      }
    );

    current_joints_subscriber = node->create_subscription<CurrentJoints>(
      "/joint/current_joints", 10,
      [this](const CurrentJoints::SharedPtr message) {
        {
          using tachimawari::joint::Joint;
          using tachimawari::joint::JointId;

          std::vector<Joint> temp_joints;
          std::set<uint8_t> joints_id;

          for (const std::string & id : {"neck_yaw", "neck_pitch"}) {
            if (JointId::by_name.find(id) != JointId::by_name.end()) {
              joints_id.insert(JointId::by_name.find(id)->second);
            }
          }

          for (const auto & joint : message->joints) {
            // Joint Id found in joints_id set
            if (joints_id.find(joint.id) != joints_id.end()) {
              temp_joints.push_back(Joint(joint.id, joint.position));
            }
          }

          this->head->set_joints(temp_joints);
        }
      }
    );

    set_joints_publisher = node->create_publisher<tachimawari_interfaces::msg::SetJoints>(
      "joint/set_joints", 10);
    set_head_publisher = node->create_publisher<atama_interfaces::msg::Head>(
      get_node_prefix() + "/set_head_data", 10);

  }
}

void HeadNode::publish_joints()
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

void HeadNode::publish_head_data()
{
  auto pan_tilt_msg = atama_interfaces::msg::Head();

  pan_tilt_msg.pan_angle = head->get_pan_angle();
  pan_tilt_msg.tilt_angle = head->get_tilt_angle();
  pan_tilt_msg.distance = head->calculate_distance_from_pan_tilt(
    head->get_pan_angle(), head->get_tilt_angle());

  set_head_publisher->publish(pan_tilt_msg);
}

void HeadNode::update()
{
  publish_joints();
  publish_head_data();
}

std::string HeadNode::get_node_prefix()
{
  return "head";
}

}  // namespace atama
