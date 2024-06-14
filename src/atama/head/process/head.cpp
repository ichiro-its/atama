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

  joints = {
    Joint(tachimawari::joint::JointId::NECK_YAW, 0.0),
    Joint(tachimawari::joint::JointId::NECK_PITCH, 0.0)
  };

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

  process();
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

  process();
}


void Head::scan(control::Command mode)
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

void Head::scan_two_direction()
{
  if (init_scanning()) {
    scan_pan_angle = get_pan_angle();
    scan_tilt_angle = get_tilt_angle();

    scan_position = (scan_mode == control::SCAN_DOWN) ? 0 : 1;

    if (pan_angle < (scan_left_limit + scan_right_limit) / 2) {
      scan_direction = 0;
      scan_pan_angle -= std::fabs(scan_right_limit);
    } else {
      scan_direction = 1;
      scan_pan_angle += std::fabs(scan_right_limit);
    }
  }

  switch (scan_position) {
    case 0:
      scan_tilt_angle = scan_bottom_limit;
      scan_pan_angle = keisan::clamp(scan_pan_angle, -70.0, 70.0);
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
      if (scan_pan_angle > ((scan_position == 0) ? 70.0 : scan_left_limit)) {
        scan_position = (scan_position + 1) % 3;
        scan_direction = 1;
      }
      break;
    case 1:
      scan_pan_angle -= scan_speed;
      if (scan_pan_angle < ((scan_position == 0) ? -70.0 : scan_right_limit)) {
        scan_position = (scan_position + 1) % 3;
        scan_direction = 0;
      }
      break;
  }
}

void Head::process()
{
  for (size_t i = 0; i < joints.size(); ++i) {
    joints[i].set_pid_gain(20.0, 3.0, 2.0);
  }

  if (is_started_scanning) {
    switch (scan_mode) {
      case control::SCAN_UP:
      case control::SCAN_DOWN:
      case control::SCAN_CUSTOM:
        scan_two_direction();
        break;
      case control::SCAN_VERTICAL:
        {
          if (init_scanning()) {
            scan_pan_angle = get_pan_angle();
            scan_tilt_angle = get_tilt_angle();
            
            scan_left_limit = 0;
            scan_right_limit = 0;

            if (get_tilt_angle() < (scan_bottom_limit + scan_top_limit) / 2) {
              scan_position = 0;
              scan_tilt_angle -= (fabs(scan_top_limit - scan_bottom_limit) / 2);
            } else {
              scan_position = 1;
              scan_tilt_angle += (fabs(scan_top_limit - scan_bottom_limit) / 2);
            }
          }

          scan_pan_angle = (scan_left_limit + scan_right_limit) / 2;
          switch (scan_position) {
            case 0:
              {
                scan_tilt_angle += scan_speed;
                if (scan_tilt_angle > scan_top_limit) {
                  scan_position = (scan_position + 1) % 2;
                }

                break;
              }
            case 1:
              {
                scan_tilt_angle -= scan_speed;
                if (scan_tilt_angle < scan_bottom_limit) {
                  scan_position = (scan_position + 1) % 2;
                }

                break;
              }
          }

          break;
        }
      case control::SCAN_HORIZONTAL:
        {
          if (init_scanning()) {
            scan_pan_angle = get_pan_angle();
            scan_tilt_angle = get_tilt_angle();

            if (pan_angle < (scan_left_limit + scan_right_limit) / 2) {
              scan_position = 0;
              scan_pan_angle -= fabs(scan_right_limit);
            } else {
              scan_position = 1;
              scan_pan_angle += fabs(scan_right_limit);
            }
          }

          scan_tilt_angle = (scan_bottom_limit + scan_top_limit) / 2;

          switch (scan_position) {
            case 0:
              {
                scan_pan_angle += scan_speed;
                if (scan_pan_angle > scan_left_limit) {
                  scan_position = (scan_position + 1) % 2;
                }

                break;
              }
            case 1:
              {
                scan_pan_angle -= scan_speed;
                if (scan_pan_angle < scan_right_limit) {
                  scan_position = (scan_position + 1) % 2;
                }

                break;
              }
          }

          break;
        }
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
  process();
}

double Head::calculate_distance_from_pan_tilt()
{
  double distance = 0.0;
  double pan = get_pan_angle();
  double tilt = get_tilt_angle();
  int coef_index = 0;
  for (int i = 0; i < pan_tilt_to_dist_coefficients.size(); i++)
  {
    double x1 = pow((pan + pan_offset), distance_regression_degrees[i][0]);
    double x2 = pow((tilt + tilt_offset), distance_regression_degrees[i][1]);

    distance += pan_tilt_to_dist_coefficients[coef_index++] * x1 * x2;
  }

  return (distance > 0.0) ? distance : 0.0;
}

double Head::calculate_tilt_from_pan_distance(double distance)
{
  double tilt = 0.0;
  double pan = get_pan_angle();
  int coef_index = 0;
  for (int i = 0; i < pan_distance_to_tilt_coefficients.size(); i++)
  {
    double x1 = pow((pan + pan_offset), distance_regression_degrees[i][0]);
    double x2 = pow((tilt + tilt_offset), distance_regression_degrees[i][1]);

    tilt += pan_distance_to_tilt_coefficients[coef_index++] * x1 * x2;
  }

  return tilt;
}

void Head::look_to_position(double goal_position_x, double goal_position_y)
{
  if (robot_position_x != -1 && robot_position_y != -1 && yaw != -1) {
    function_id = control::LOOK_TO_POSITION;
    float dx = goal_position_x - robot_position_x;
    float dy = goal_position_y - robot_position_y;

    float pan = yaw - keisan::signed_arctan(dy, dx).normalize().degree();
    float tilt = calculate_tilt_from_pan_distance(std::hypot(dx, dy));

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
          throw ex;
        }
      } else if (key == "Offset") {
        try {
          val.at("pan_offset").get_to(pan_offset);
          val.at("tilt_offset").get_to(tilt_offset);
        } catch (nlohmann::json::parse_error & ex) {
          std::cerr << "parse error at byte " << ex.byte << std::endl;
          throw ex;
        }
      } else if (key == "Limit") {
        try {
          val.at("left_limit").get_to(left_limit);
          val.at("right_limit").get_to(right_limit);
          val.at("top_limit").get_to(top_limit);
          val.at("bottom_limit").get_to(bottom_limit);
        } catch (nlohmann::json::parse_error & ex) {
          std::cerr << "parse error at byte " << ex.byte << std::endl;
          throw ex;
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
          throw ex;
        }
      } else if (key == "Gain") {
        try {
          val.at("pan_p_gain").get_to(pan_p_gain);
          val.at("pan_d_gain").get_to(pan_d_gain);
          val.at("tilt_p_gain").get_to(tilt_p_gain);
          val.at("tilt_d_gain").get_to(tilt_d_gain);
        } catch (nlohmann::json::parse_error & ex) {
          std::cerr << "parse error at byte " << ex.byte << std::endl;
          throw ex;
        }
      } else if (key == "DistanceRegression") {
        try {
          val.at("distance_polynomial_coefficients").get_to(pan_tilt_to_dist_coefficients);
          val.at("tilt_polynomial_coefficients").get_to(pan_distance_to_tilt_coefficients);
          val.at("distance_polynomial_degrees").get_to(distance_regression_degrees);
        } catch (nlohmann::json::parse_error & ex) {
          std::cerr << "parse error at byte " << ex.byte << std::endl;
          throw ex;
        }
      } else if (key == "FOVCamera") {
        try {
          val.at("horizontal_fov").get_to(horizontal_fov);
          val.at("vertical_fov").get_to(vertical_fov);
        } catch (nlohmann::json::parse_error & ex) {
          std::cerr << "parse error at byte " << ex.byte << std::endl;
          throw ex;
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
        
        object_center_x = item.left + item.right / 2;
        object_center_y = item.top + item.bottom / 2;
      }
    }
  } else {
    return;
  }

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
      keisan::Point2 offset = calculate_angle_offset_from_pixel(object_center_x, object_center_y);

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

keisan::Point2 Head::calculate_object_position_from_pixel(double pixel_x, double pixel_y, bool is_ball)
{
  keisan::Point2 object_position_from_pixel = keisan::Point2(-1, -1);

  // calculate object distance from pixels and center distance
  double horizontal_degree = horizontal_fov * pixel_x;
  double vertical_degree = vertical_fov * pixel_y;
  double center_real_distance = is_ball ? Head::calculate_distance_from_pan_tilt() : Head::calculate_distance_from_pan_tilt() + 7;

  if (Head::calculate_distance_from_pan_tilt() > -1) {
    double camera_height = center_real_distance / keisan::make_radian((90 + Head::get_tilt_angle())).tan();
    double x_relative_from_camera = camera_height * keisan::make_radian((90 + Head::get_tilt_angle() - vertical_degree)).tan();
    double y_relative_from_camera = x_relative_from_camera * keisan::make_radian(horizontal_degree).tan();
    double object_distance_from_camera = sqrt(pow(x_relative_from_camera, 2) + pow(y_relative_from_camera, 2));

    keisan::Angle<double> object_delta_direction = keisan::make_radian((yaw - Head::get_pan_angle() + horizontal_degree));

    object_position_from_pixel.x = object_distance_from_camera * object_delta_direction.cos();
    object_position_from_pixel.y = object_distance_from_camera * object_delta_direction.sin();
  }

  return object_position_from_pixel;
}

void Head::track_pixel(double pixel_x, double pixel_y)
{
  if (!check_time_belief()) {
    return;
  }

  function_id = control::TRACK_OBJECT;
  stop_scan();

  keisan::Point2 offset = calculate_angle_offset_from_pixel(pixel_x, pixel_y);

  tracking(offset.x, offset.y);
}

keisan::Point2 Head::calculate_angle_offset_from_pixel(double pixel_x, double pixel_y)
{
  keisan::Point2 offset(-1, -1);
  offset.x = ((pixel_x / static_cast<float>(camera_width)) - 0.5) * view_h_angle;
  offset.y = ((pixel_y / static_cast<float>(camera_height)) - 0.5) * view_v_angle;
  offset *= -1;

  return offset;
}  

} // namespace atama
