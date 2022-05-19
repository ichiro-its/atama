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
      "ninshiki_py/detection", 10,
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

    {
      using atama_interfaces::srv::RunHead;
      run_head_service = node->create_service<RunHead>(
        get_node_prefix() + "/run_head",
        [this](std::shared_ptr<RunHead::Request> request,
        std::shared_ptr<RunHead::Response> response) {
          rclcpp::Rate rcl_rate(15ms);

          // Assume the function is not exist
          bool is_function_exist = false;

          // Check existence of the function
          switch (request->function_id) {
            case control::SCAN_CUSTOM:
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
            case control::TRACK_OBJECT:
              {
                this->head->object_name = request->track_param.object_name;
                is_function_exist = true;
                break;
              }
            case control::MOVE_BY_ANGLE:
              {
                this->head->pan_angle_goal = request->move_by_angle_param.pan_angle;
                this->head->tilt_angle_goal = request->move_by_angle_param.tilt_angle;
                is_function_exist = true;
                break;
              }
            case control::LOOK_TO_POSITION:
              {
                this->head->goal_position_x = request->look_to_param.goal_position_x;
                this->head->goal_position_y = request->look_to_param.goal_position_y;
                is_function_exist = true;
                break;
              }
            case control::SCAN_UP:
            case control::SCAN_DOWN:
            case control::SCAN_VERTICAL:
            case control::SCAN_HORIZONTAL:
            case control::SCAN_MARATHON:
              {
                is_function_exist = true;
                break;
              }
          }

          if (is_function_exist)
            req_function_id = request->function_id;

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

void HeadNode::process(int function_id)
{
  if (!head->is_joint_empty()) {
    switch (function_id) {
      case control::SCAN_UP: head->scan_up(); break;
      case control::SCAN_DOWN: head->scan_down(); break;
      case control::SCAN_VERTICAL: head->scan_vertical(); break;
      case control::SCAN_HORIZONTAL: head->scan_horizontal(); break;
      case control::SCAN_MARATHON: head->scan_marathon(); break;
      case control::SCAN_CUSTOM: head->scan_custom(control::SCAN_CUSTOM); break;
      case control::TRACK_OBJECT:
        head->track_object(head->object_name);
        break;
      case control::MOVE_BY_ANGLE:
        head->move_by_angle(
          head->pan_angle_goal,
          head->tilt_angle_goal);
        break;
      // TODO(nathan): implement look_to_position() after
      // robot_position_x, robot_position_y, yaw are obtained
      case control::LOOK_TO_POSITION:
        break;
    }
  }

  publish_joints();
  publish_head_data();
}

bool HeadNode::is_detection_result_empty()
{
  std::set<std::string> result_name;
  for (const auto & name : head->detection_result) {
    result_name.insert(name.label);
  }

  // Object not found
  return result_name.find(head->object_name) == result_name.end();
}

bool HeadNode::check_move_by_angle()
{
  return head->get_pan_angle() == head->pan_angle_goal &&
         head->get_tilt_angle() == head->tilt_angle_goal;
}

bool HeadNode::check_process_is_finished()
{
  switch (req_function_id) {
    case control::SCAN_UP:
    case control::SCAN_DOWN:
    case control::SCAN_VERTICAL:
    case control::SCAN_HORIZONTAL:
    case control::SCAN_MARATHON:
    case control::SCAN_CUSTOM: return !is_detection_result_empty(); break;
    case control::TRACK_OBJECT: return is_detection_result_empty(); break;
    case control::MOVE_BY_ANGLE: return check_move_by_angle(); break;
    // TODO(nathan): implement look_to_position() after
    // robot_position_x, robot_position_y, yaw are obtained
    case control::LOOK_TO_POSITION: break;
  }
}

std::string HeadNode::get_node_prefix()
{
  return "head";
}

}  // namespace atama
