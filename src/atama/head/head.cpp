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

#include <cmath>
#include <fstream>
#include <memory>
#include <nlohmann/json.hpp>
#include <sstream>
#include <string>

#include "atama/head/head.hpp"
#include "common/algebra.h"

namespace atama
{

Head::Head(double position_x, double position_y, float yaw)
{
  this->position_x = position_x;
  this->position_y = position_y;
  this->yaw = yaw;

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

  scan_speed = 0.5;

  no_ball_count = 0;
  ball_count = 0;

  // m_Joint.Set_enableHead_only(true);
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
  pan_angle = current_pan_angle;
  tilt_angle = -current_tilt_angle;

  pan_error = 0;
  pan_error_difference = 0;

  tilt_error = 0;
  tilt_error_difference = 0;
}

void Head::move_by_angle(double pan_angle, double tilt_angle)
{
  stop_scan();
  this->pan_angle = pan_center + alg::clampValue(pan_angle, right_limit, left_limit);
  this->tilt_angle = tilt_center + alg::clampValue(tilt_angle, bottom_limit, top_limit);
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
    pan_angle = current_pan_angle;
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
    tilt_angle = current_tilt_angle;
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

  pan_angle = pan_center + alg::clampValue(pan_angle - pan_center, right_limit, left_limit);
  tilt_angle = tilt_center + alg::clampValue(tilt_angle - tilt_center, bottom_limit, top_limit);
}

void Head::process()
{
  for (int i = 0; i < static_cast<int>(joints.size()); i++) {
    joints[i].set_pid_gain(20.0, 3.0, 2.0);
  }

  if (is_started_scanning) {
    switch (scan_mode) {
      case SCAN_UP:
      case SCAN_DOWN:
        {
          if (init_scanning()) {
            scan_left_limit = 60.0;
            scan_right_limit = -60.0;

            scan_top_limit = 0.0;
            scan_bottom_limit = -75.0;

            scan_speed = 0.6;

            scan_pan_angle = get_pan_angle();
            scan_tilt_angle = get_tilt_angle();

            scan_position = (scan_mode == SCAN_DOWN) ? 0 : 1;

            if (pan_angle < (scan_left_limit + scan_right_limit) / 2) {
              scan_direction = 0;
              scan_pan_angle -= 30.0;
            } else {
              scan_direction = 1;
              scan_pan_angle += 30.0;
            }
          }

          switch (scan_position) {
            case 0: scan_tilt_angle = scan_bottom_limit; break;
            case 1: scan_tilt_angle = (scan_bottom_limit + scan_top_limit) * 0.5; break;
            case 2: scan_tilt_angle = scan_top_limit; break;
          }

          switch (scan_direction) {
            case 0:
              {
                scan_pan_angle += scan_speed;
                if (scan_pan_angle > scan_left_limit) {
                  scan_position = (scan_position + 1) % 3;
                  scan_direction = 1;
                }

                break;
              }

            case 1:
              {
                scan_pan_angle -= scan_speed;
                if (scan_pan_angle < scan_right_limit) {
                  scan_position = (scan_position + 1) % 3;
                  scan_direction = 0;
                }

                break;
              }
          }

          break;
        }

      case SCAN_VERTICAL:
        {
          if (init_scanning()) {
            scan_left_limit = 0.0;
            scan_right_limit = 0.0;

            scan_top_limit = 0.0;
            scan_bottom_limit = -70.0;

            scan_speed = 0.8;

            scan_pan_angle = get_pan_angle();
            scan_tilt_angle = get_tilt_angle();

            if (get_tilt_angle() < (scan_bottom_limit + scan_top_limit) / 2) {
              scan_position = 0;
              scan_tilt_angle -= 15.0;
            } else {
              scan_position = 1;
              scan_tilt_angle += 15.0;
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

      case SCAN_HORIZONTAL:
        {
          if (init_scanning()) {
            scan_left_limit = 70.0;
            scan_right_limit = -70.0;

            scan_top_limit = -30.0;
            scan_bottom_limit = -30.0;

            scan_pan_angle = get_pan_angle();
            scan_tilt_angle = get_tilt_angle();

            scan_speed = 0.8;

            if (pan_angle < (scan_left_limit + scan_right_limit) / 2) {
              scan_position = 0;
              scan_pan_angle -= 30.0;
            } else {
              scan_position = 1;
              scan_pan_angle += 30.0;
            }
          }

          scan_tilt_angle = -60.0;

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

    pan_angle = pan_center + alg::clampValue(scan_pan_angle, right_limit, left_limit);
    tilt_angle = tilt_center + alg::clampValue(scan_tilt_angle, bottom_limit, top_limit);
  }

  if (!joints.empty())
  {
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

double Head::calculate_distance_from_pan_tilt(double pan, double tilt)
{
  double distance = 0.0;

  for (int i = 6; i >= 0; i--) {
    for (int j = 0; j <= i; j++) {
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

  for (int i = 0; i <= 4; i++) {
    double x2 = pow((tilt + tilt_center + tilt_offset), i);
    distance += (tilt_to_distance_[i]) * x2;
  }

  return (distance > 0.0) ? distance : 0.0;
}

double Head::calculate_tilt_from_pan_distance(double pan, double distance)
{
  double tilt = 0.0;

  for (int i = 4; i >= 0; i--) {
    for (int j = 0; j <= i; j++) {
      double x1 = pow((pan + pan_center + pan_offset), (i - j));
      double x2 = pow(distance, j);
      tilt += (pan_distance_to_tilt_[i - j][j]) * x1 * x2;
    }
  }

  return ((tilt < 15.0) ? tilt : 15.0) - tilt_center - tilt_offset;
}

void Head::look_to_position(double position_x, double position_y)
{
  float dx = position_x - this->position_x;
  float dy = position_y - this->position_y;

  float pan = yaw - (alg::direction(dx, dy) * alg::rad2Deg());
  float tilt = calculate_tilt_from_pan_distance(alg::distance(dx, dy));

  move_by_angle(pan - pan_center, tilt);
}

void Head::set_pan_tilt_angle(double pan, double tilt)
{
  current_pan_angle = pan;
  current_tilt_angle = tilt;
}

void Head::load_data(std::string file_name)
{
  std::ifstream file(file_name + "head/" + "head.json");
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
        for (int i = 6; i >= 0; i--) {
          for (int j = 0; j <= i; j++) {
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
        for (int i = 0; i <= 4; i++) {
          char buffer[32];
          snprintf(buffer, sizeof(buffer), "pan_%d_tilt_%d", 0, i);
          val.at(buffer).get_to(tilt_to_distance_[i]);
        }
      } catch (nlohmann::json::parse_error & ex) {
        std::cerr << "parse error at byte " << ex.byte << std::endl;
      }
    } else if (key == "PanDistanceToTilt") {
      try {
        for (int i = 4; i >= 0; i--) {
          for (int j = 0; j <= i; j++) {
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
}

// void Head::track_ball(
//   std::shared_ptr<CameraMeasurement> camera,
//   keisan::Point2 pos, float view_v_angle, float view_h_angle)
// {
//   stop_scan();
//   if (pos.x == 0 || pos.y == 0) {
//     ball_position.x = -1;
//     ball_position.y = -1;
//     ball_count = 0;
//     if (no_ball_count < no_ball_max_count) {
//       tracking();
//       no_ball_count++;
//     } else {
//       initialize();
//     }
//   } else {
//     no_ball_count = 0;
//     if (ball_count < ball_max_count) {
//       ball_count++;
//     } else {
//       keisan::Point2 center = keisan::Point2(camera->width() / 2, camera->height() / 2);
//       keisan::Point2 offset = pos - center;
//       offset *= -1;
//       offset.x *= (view_v_angle / camera->width());
//       offset.y *= (view_h_angle / camera->height());

//       ball_position = offset;
//       tracking(ball_position.x, ball_position.y);
//     }
//   }
// }

}  // namespace atama
