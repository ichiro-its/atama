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

#include <memory>
#include <string>

#include "atama/head/control/node/control_node.hpp"

#include "nlohmann/json.hpp"
#include "rclcpp/rclcpp.hpp"
#include "atama/head/control/helper/command.hpp"
#include "atama/head/process/head.hpp"

using std::placeholders::_1;

namespace atama::control
{

ControlNode::ControlNode(
  rclcpp::Node::SharedPtr node, std::shared_ptr<Head> head)
: node(node), head(head), process([]() {return false;})
{
  run_head_subscriber = node->create_subscription<RunHead>(
    get_node_prefix() + "/run_head", 10,
    std::bind(&ControlNode::run_head_callback, this, _1));

  status_publisher = node->create_publisher<Status>(
    get_node_prefix() + "/status", 10);
}

void ControlNode::run_head_callback(const RunHead::SharedPtr message)
{
  auto parameters = nlohmann::json::parse(message->parameters);

  switch (message->command) {
    case Command::SCAN_UP:
      {
        process = [this]() {
          this->head->scan_up();
          return is_object_name_not_in_detection_result();
        };

        break;
      }
  
    case Command::SCAN_DOWN:
      {
        process = [this]() {
          this->head->scan_down();
          return is_object_name_not_in_detection_result();
        };

        break;
      }

    case Command::SCAN_VERTICAL:
      {
        process = [this]() {
          this->head->scan_vertical();
          return is_object_name_not_in_detection_result();
        };

        break;
      }

    case Command::SCAN_HORIZONTAL:
      {
        process = [this]() {
          this->head->scan_horizontal();
          return is_object_name_not_in_detection_result();
        };

        break;
      }
  
    case Command::SCAN_MARATHON:
      {
        process = [this]() {
          this->head->scan_marathon();
          return is_object_name_not_in_detection_result();
        };

        break;
      }

    case Command::SCAN_CUSTOM:
      {
        double left_limit, right_limit, top_limit, bottom_limit;

        for (auto &[key, val] : parameters.items()) {
          if (key == "left_limit") {
            left_limit = val.get<double>();
          } else if (key == "right_limit") {
            right_limit = val.get<double>();
          } else if (key == "top_limit") {
            top_limit = val.get<double>();
          } else if (key == "bottom_limit") {
            bottom_limit = val.get<double>();
          }
        }

        process = [this, left_limit, right_limit, top_limit, bottom_limit]() {
          this->head->set_scan_limit(left_limit, right_limit, top_limit, bottom_limit);
          this->head->scan_custom();
          return !is_object_name_not_in_detection_result();
        };

        break;
      }

    case Command::TRACK_OBJECT:
      {
        std::string object_name;

        for (auto &[key, val] : parameters.items()) {
          if (key == "object_name") {
            object_name = val.get<double>();
          } 
        }

        process = [this, object_name]() {
          this->head->track_object(object_name);
          return is_object_name_not_in_detection_result();
        };

        break;
      }

    case Command::MOVE_BY_ANGLE:
      {
        double pan, tilt;

        for (auto &[key, val] : parameters.items()) {
          if (key == "pan_angle") {
            pan = val.get<double>();
          } else if (key == "tilt_angle") {
            tilt = val.get<double>();
          }
        }

        process = [this, pan, tilt]() {
          this->head->move_by_angle(pan, tilt);
          return check_move_by_angle();
        };

        break;
      }

    case Command::LOOK_TO_POSITION:
      {
        double goal_x, goal_y;

        for (auto &[key, val] : parameters.items()) {
          if (key == "goal_position_x") {
            goal_x = val.get<double>();
          } else if (key == "goal_position_y") {
            goal_y = val.get<double>();
          } 
        }

        process = [this, goal_x, goal_y]() {
          this->head->look_to_position(goal_x, goal_y);
          return check_move_by_angle();
        };

        break;
      }
  }
}

void ControlNode::update()
{
  auto status_msg = Status();
  status_msg.status = process();

  status_publisher->publish(status_msg);
}

bool ControlNode::is_object_name_not_in_detection_result()
{
  std::set<std::string> result_name;
  for (const auto & name : this->head->detection_result) {
    result_name.insert(name.label);
  }

  // If object not found, return true
  return result_name.find(this->head->object_name) == result_name.end();
}

bool ControlNode::check_move_by_angle()
{
  return this->head->get_pan_angle() == this->head->pan_angle_goal &&
         this->head->get_tilt_angle() == this->head->tilt_angle_goal;
}

std::string ControlNode::get_node_prefix() const
{
  return "atama/control";
}

}  // namespace atama::control
