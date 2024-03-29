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

#include <nlohmann/json.hpp>

#include <cmath>
#include <fstream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include "atama/head/process/head.hpp"

namespace atama
{

Head::Head()
{
  is_started_scanning = false;
  pan_only = false;
  tilt_only = false;

  scan_mode = 0;
  scan_init = false;

  scan_pan_angle = pan_center;
  scan_tilt_angle = tilt_center;

  scan_left_limit = 90;
  scan_right_limit = -90;
  scan_top_limit = 55;
  scan_bottom_limit = -15;

  scan_speed = 0.35;

  no_object_count = 0;
  object_count = 0;
  object_name = "ball";

  joints = {};
  initiate_min_time = true;

  camera_width = -1;
  camera_height = -1;
  view_v_angle = -1;
  view_h_angle = -1;

  function_id = control::NONE;
  prev_function_id = control::NONE;
  detection_result.clear();

  robot_position_x = -1;
  robot_position_y = -1;
  yaw = -1;
}

bool Head::init_scanning()
{
  if (scan_init) {
    return false;
  }

  return scan_init = true;
}

void Head::initialize()
{
  pan_angle = 0;
  tilt_angle = 0;

  pan_error = 0;
  pan_error_difference = 0;

  tilt_error = 0;
  tilt_error_difference = 0;
}

void Head::move_by_angle(double pan_angle, double tilt_angle)
{
  function_id = control::MOVE_BY_ANGLE;
  stop_scan();
  this->pan_angle = pan_center + keisan::clamp(pan_angle, right_limit, left_limit);
  this->tilt_angle = tilt_center + keisan::clamp(tilt_angle, bottom_limit, top_limit);

  pan_angle_goal = pan_angle;
  tilt_angle_goal = tilt_angle;

  // set calculation result
  if (!joints.empty() && joints.size() >= 2) {
    joints[0].set_position(pan_angle);
    joints[1].set_position(tilt_angle);
  }
}

void Head::tracking(double pan, double tilt)
{
  pan_error_difference = pan - pan_error;
  pan_error = pan;

  tilt_error_difference = tilt - tilt_error;
  tilt_error = tilt;

  pan_only = false;
  tilt_only = false;

  tracking();
}

void Head::tracking_pan_only(double pan)
{
  pan_error_difference = pan - pan_error;
  pan_error = pan;

  pan_only = true;

  tracking();
}

void Head::tracking_tilt_only(double tilt)
{
  tilt_error_difference = tilt - tilt_error;
  tilt_error = tilt;

  tilt_only = true;

  tracking();
}

void Head::tracking()
{
  stop_scan();

  if (tilt_only) {
    pan_angle = joints[0].get_position();
  } else {
    double p_offset = pan_error * pan_p_gain;
    p_offset *= p_offset;
    if (pan_error < 0) {
      p_offset = -p_offset;
    }

    double d_offset = pan_error_difference * pan_d_gain;
    d_offset *= d_offset;
    if (pan_error_difference < 0) {
      d_offset = -d_offset;
    }

    pan_angle += (p_offset + d_offset);
  }

  if (pan_only) {
    tilt_angle = joints[1].get_position();
  } else {
    double p_offset = tilt_error * tilt_p_gain;
    p_offset *= p_offset;
    if (tilt_error < 0) {
      p_offset = -p_offset;
    }

    double d_offset = tilt_error_difference * tilt_d_gain;
    d_offset *= d_offset;
    if (tilt_error_difference < 0) {
      d_offset = -d_offset;
    }
    tilt_angle += (p_offset + d_offset);
  }

  pan_angle = pan_center + keisan::clamp(pan_angle - pan_center, right_limit, left_limit);
  tilt_angle = tilt_center + keisan::clamp(tilt_angle - tilt_center, bottom_limit, top_limit);

  // set calculation result
  if (!joints.empty() && joints.size() >= 2) {
    joints[0].set_position(pan_angle);
    joints[1].set_position(tilt_angle);
  }
}


void Head::scan(int mode)
{
  start_scan();

  if (scan_mode == mode) {
    return;
  }

  scan_mode = mode;
  scan_init = false;
}

void Head::set_scan_limit(
  double left_limit, double right_limit,
  double top_limit, double bottom_limit)
{
  scan_left_limit = left_limit;
  scan_right_limit = right_limit;
  scan_top_limit = top_limit;
  scan_bottom_limit = bottom_limit;
}

void Head::scan_one_direction()
{
  if (init_scanning()) {
    double value_change;

    scan_pan_angle = get_pan_angle();
    scan_tilt_angle = get_tilt_angle();

    if (scan_mode == control::SCAN_VERTICAL) {
      value_change = 15.0;
    } else {
      value_change = 30.0;
    }

    if (get_tilt_angle() < (scan_bottom_limit + scan_top_limit) / 2) {
      scan_position = 0;
      scan_tilt_angle -= value_change;
    } else {
      scan_position = 1;
      scan_tilt_angle += value_change;
    }
  }

  if (scan_mode == control::SCAN_VERTICAL) {
    scan_pan_angle = (scan_left_limit + scan_right_limit) / 2;
    switch (scan_position) {
      case 0:
        scan_tilt_angle += scan_speed;
        if (scan_tilt_angle > scan_top_limit) {
          scan_position = (scan_position + 1) % 2;
        }
        break;
      case 1:
        scan_tilt_angle -= scan_speed;
        if (scan_tilt_angle < scan_bottom_limit) {
          scan_position = (scan_position + 1) % 2;
        }
        break;
    }
  } else {
    scan_tilt_angle = -60.0;
    switch (scan_position) {
      case 0:
        scan_pan_angle += scan_speed;
        if (scan_pan_angle > scan_left_limit) {
          scan_position = (scan_position + 1) % 2;
        }

        break;
      case 1:
        scan_pan_angle -= scan_speed;
        if (scan_pan_angle < scan_right_limit) {
          scan_position = (scan_position + 1) % 2;
        }

        break;
    }
  }
}

void Head::scan_two_direction()
{
  if (init_scanning()) {
    scan_pan_angle = get_pan_angle();
    scan_tilt_angle = get_tilt_angle();

    scan_position = (scan_mode == control::SCAN_DOWN) ? 0 : 1;

    if (pan_angle < (scan_left_limit + scan_right_limit) / 2) {
      scan_direction = 0;
      scan_pan_angle -= 30.0;
    } else {
      scan_direction = 1;
      scan_pan_angle += 30.0;
    }
  }

  switch (scan_position) {
    case 0:
      scan_tilt_angle = scan_bottom_limit;
      break;
    case 1:
      scan_tilt_angle = (scan_bottom_limit + scan_top_limit) * 0.5;
      break;
    case 2:
      scan_tilt_angle = scan_top_limit;
      break;
  }

  switch (scan_direction) {
    case 0:
      scan_pan_angle += scan_speed;
      if (scan_pan_angle > scan_left_limit) {
        scan_position = (scan_position + 1) % 3;
        scan_direction = 1;
      }
      break;
    case 1:
      scan_pan_angle -= scan_speed;
      if (scan_pan_angle < scan_right_limit) {
        scan_position = (scan_position + 1) % 3;
        scan_direction = 0;
      }
      break;
  }
}

void Head::scan_process()
{
  for (size_t i = 0; i < joints.size(); ++i) {
    joints[i].set_pid_gain(20.0, 3.0, 2.0);
  }

  if (is_started_scanning) {
    switch (scan_mode) {
      case control::SCAN_UP:
      case control::SCAN_DOWN:
      case control::SCAN_CUSTOM: scan_two_direction(); break;
      case control::SCAN_VERTICAL:
      case control::SCAN_HORIZONTAL: scan_one_direction(); break;
    }

    pan_angle = pan_center + keisan::clamp(scan_pan_angle, right_limit, left_limit);
    tilt_angle = tilt_center + keisan::clamp(scan_tilt_angle, bottom_limit, top_limit);
  }

  // set calculation result
  if (!joints.empty() && joints.size() >= 2) {
    joints[0].set_position(pan_angle);
    joints[1].set_position(tilt_angle);
  }
}

void Head::scan_custom(control::Command scan_type)
{
  if (!check_time_belief()) {
    return;
  }

  function_id = scan_type;
  scan(scan_type);
  scan_process();
}

double Head::calculate_distance_from_pan_tilt(double pan, double tilt)
{
  double distance = 0.0;

  for (int i = 6; i >= 0; --i) {
    for (int j = 0; j <= i; ++j) {
      double x1 = pow((pan + pan_center + pan_offset), (i - j));
      double x2 = pow((tilt + tilt_center + tilt_offset), j);
      distance += (pan_tilt_to_distance_[i - j][j]) * x1 * x2;
    }
  }

  return (distance > 0.0) ? distance : 0.0;
}

double Head::calculate_distance_from_tilt(double tilt)
{
  double distance = 0.0;

  for (int i = 0; i <= 4; ++i) {
    double x2 = pow((tilt + tilt_center + tilt_offset), i);
    distance += (tilt_to_distance_[i]) * x2;
  }

  return (distance > 0.0) ? distance : 0.0;
}

double Head::calculate_tilt_from_pan_distance(double pan, double distance)
{
  double tilt = 0.0;

  for (int i = 4; i >= 0; --i) {
    for (int j = 0; j <= i; ++j) {
      double x1 = pow((pan + pan_center + pan_offset), (i - j));
      double x2 = pow(distance, j);
      tilt += (pan_distance_to_tilt_[i - j][j]) * x1 * x2;
    }
  }

  return ((tilt < 15.0) ? tilt : 15.0) - tilt_center - tilt_offset;
}

void Head::look_to_position(double goal_position_x, double goal_position_y)
{
  if (robot_position_x != -1 && robot_position_y != -1 && yaw != -1) {
    function_id = control::LOOK_TO_POSITION;
    float dx = goal_position_x - robot_position_x;
    float dy = goal_position_y - robot_position_y;

    float pan = yaw - keisan::signed_arctan(dy, dx).normalize().degree();
    float tilt = calculate_tilt_from_pan_distance(keisan::signed_arctan(dy, dx).degree());

    move_by_angle(pan - pan_center, tilt);
  }
}

void Head::set_pan_tilt_angle(double pan, double tilt)
{
  current_pan_angle = pan;
  current_tilt_angle = tilt;
}

void Head::load_config(const std::string & file_name)
{
  try {
    std::ifstream file(file_name + "head.json");
    nlohmann::json walking_data = nlohmann::json::parse(file);

    for (auto &[key, val] : walking_data.items()) {
      if (key == "Center") {
        try {
          val.at("pan_center").get_to(pan_center);
          val.at("tilt_center").get_to(tilt_center);
        } catch (nlohmann::json::parse_error & ex) {
          std::cerr << "parse error at byte " << ex.byte << std::endl;
        }
      } else if (key == "Offset") {
        try {
          val.at("pan_offset").get_to(pan_offset);
          val.at("tilt_offset").get_to(tilt_offset);
        } catch (nlohmann::json::parse_error & ex) {
          std::cerr << "parse error at byte " << ex.byte << std::endl;
        }
      } else if (key == "Limit") {
        try {
          val.at("left_limit").get_to(left_limit);
          val.at("right_limit").get_to(right_limit);
          val.at("top_limit").get_to(top_limit);
          val.at("bottom_limit").get_to(bottom_limit);
        } catch (nlohmann::json::parse_error & ex) {
          std::cerr << "parse error at byte " << ex.byte << std::endl;
        }
      } else if (key == "Scan") {
        try {
          val.at("scan_speed").get_to(scan_speed);
          val.at("left_limit").get_to(scan_left_limit);
          val.at("right_limit").get_to(scan_right_limit);
          val.at("top_limit").get_to(scan_top_limit);
          val.at("bottom_limit").get_to(scan_bottom_limit);
        } catch (nlohmann::json::parse_error & ex) {
          std::cerr << "parse error at byte " << ex.byte << std::endl;
        }
      } else if (key == "Gain") {
        try {
          val.at("pan_p_gain").get_to(pan_p_gain);
          val.at("pan_d_gain").get_to(pan_d_gain);
          val.at("tilt_p_gain").get_to(tilt_p_gain);
          val.at("tilt_d_gain").get_to(tilt_d_gain);
        } catch (nlohmann::json::parse_error & ex) {
          std::cerr << "parse error at byte " << ex.byte << std::endl;
        }
      } else if (key == "PanTiltToDistance") {
        try {
          for (int i = 6; i >= 0; --i) {
            for (int j = 0; j <= i; ++j) {
              char buffer[32];
              snprintf(buffer, sizeof(buffer), "pan_%d_tilt_%d", (i - j), j);
              val.at(buffer).get_to(pan_tilt_to_distance_[i - j][j]);
            }
          }
        } catch (nlohmann::json::parse_error & ex) {
          std::cerr << "parse error at byte " << ex.byte << std::endl;
        }
      } else if (key == "TiltToDistance") {
        try {
          for (int i = 0; i <= 4; ++i) {
            char buffer[32];
            snprintf(buffer, sizeof(buffer), "pan_%d_tilt_%d", 0, i);
            val.at(buffer).get_to(tilt_to_distance_[i]);
          }
        } catch (nlohmann::json::parse_error & ex) {
          std::cerr << "parse error at byte " << ex.byte << std::endl;
        }
      } else if (key == "PanDistanceToTilt") {
        try {
          for (int i = 4; i >= 0; --i) {
            for (int j = 0; j <= i; ++j) {
              char buffer[32];
              snprintf(buffer, sizeof(buffer), "pan_%d_distance_%d", (i - j), j);
              val.at(buffer).get_to(pan_distance_to_tilt_[i - j][j]);
            }
          }
        } catch (nlohmann::json::parse_error & ex) {
          std::cerr << "parse error at byte " << ex.byte << std::endl;
        }
      }
    }
  } catch (nlohmann::json::parse_error & ex) {
    // TODO(nathan): will be used for logging
  }
}

void Head::track_object(const std::string & object_name)
{
  if (!check_time_belief()) {
    return;
  }

  function_id = control::TRACK_OBJECT;
  stop_scan();

  // filter the object by its label
  std::vector<ninshiki_interfaces::msg::DetectedObject> filtered_result;
  if (!detection_result.empty()) {
    for (const auto & item : detection_result) {
      if (item.label == object_name) {
        filtered_result.push_back(item);
      }
    }
  } else {
    return;
  }

  // looking at the center of the object
  // Pick an object with the biggest confidence
  double confidence = 0.0;
  double object_center_x, object_center_y;
  if (!filtered_result.empty()) {
    for (const auto & item : detection_result) {
      if (item.score > confidence) {
        confidence = item.score;

        object_center_x = (item.left * camera_width +
          item.right * camera_width) / 2;
        object_center_y = (item.top * camera_height +
          item.bottom * camera_height) / 2;
      }
    }
  } else {
    return;
  }
  keisan::Point2 pos(object_center_x, object_center_y);

  // There is no object with the label we want
  if (filtered_result.empty()) {
    object_count = 0;
    if (no_object_count < no_object_max_count) {
      tracking();
      ++no_object_count;
    } else {
      initialize();
    }
  } else {
    no_object_count = 0;
    if (object_count < object_max_count) {
      ++object_count;
    } else {
      keisan::Point2 center = keisan::Point2(camera_width, camera_height) / 2;
      keisan::Point2 offset = pos - center;
      offset *= -1;
      offset.x *= (view_h_angle / camera_width);
      offset.y *= (view_v_angle / camera_height);

      tracking(offset.x, offset.y);
    }
  }

  detection_result.clear();
}

bool Head::check_time_belief()
{
  // return true for the first time any function that is called
  if (initiate_min_time) {
    initiate_min_time = false;
    prev_function_id = function_id;
  } else {
    // start count 0.5 second for time belief when the function that is called
    // is different from the previous
    if (prev_function_id != function_id) {
      prev_function_id = function_id;
      min_time = std::chrono::system_clock::now() + std::chrono::milliseconds(500);
      return false;
    } else {
      // when function that is called is same with previous
      if (std::chrono::system_clock::now() < min_time) {
        return false;
      }
    }
  }

  return true;
}

void Head::set_joints(std::vector<Joint> joints_param)
{
  joints.clear();
  for (const auto & joint : joints_param) {
    joints.push_back(Joint(joint.get_id(), joint.get_position()));
  }

  pan_angle = joints[0].get_position();
  tilt_angle = joints[1].get_position();
}

}  // namespace atama
