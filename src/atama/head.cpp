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

#include <aruku/walking.hpp>
#include <atama/head.hpp>
#include <kansei/imu.hpp>

#include <cmath>
#include <fstream>
#include <memory>
#include <string>
#include <sstream>

#include "common/algebra.h"

namespace atama
{

Head::Head(std::shared_ptr<aruku::Walking> walking, std::shared_ptr<kansei::Imu> imu)
{
  walking = walking;
  imu = imu;

  is_started = false;
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

  for (auto id : {"neck_yaw", "neck_pitch"}) {
    tachimawari::Joint joint(id);
    joints.push_back(joint);
  }

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
  stop();

  pan_angle = pan_center + alg::clampValue(pan_angle, right_limit, left_limit);
  tilt_angle = tilt_center + alg::clampValue(tilt_angle, bottom_limit, top_limit);
}

void Head::move_tracking(double pan, double tilt)
{
  pan_error_difference = pan - pan_error;
  pan_error = pan;

  tilt_error_difference = tilt - tilt_error;
  tilt_error = tilt;

  pan_only = false;
  tilt_only = false;

  move_tracking();
}

void Head::move_tracking_pan_only(double pan)
{
  pan_error_difference = pan - pan_error;
  pan_error = pan;

  pan_only = true;

  move_tracking();
}

void Head::move_tracking_tilt_only(double tilt)
{
  tilt_error_difference = tilt - tilt_error;
  tilt_error = tilt;

  tilt_only = true;

  move_tracking();
}

void Head::move_tracking()
{
  stop();

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
  for (int i = 0; i < joints.size(); i++) {
    joints[i].set_pid_gain(20.0, 3.0, 2.0);
  }

  if (is_started) {
    switch (scan_mode) {
      case SCAN_BALL_UP:
      case SCAN_BALL_DOWN:
        {
          if (init_scanning()) {
            scan_left_limit = 60.0;
            scan_right_limit = -60.0;

            scan_top_limit = 0.0;
            scan_bottom_limit = -75.0;

            scan_speed = 0.2;

            scan_pan_angle = get_pan_angle();
            scan_tilt_angle = get_tilt_angle();

            scan_position = (scan_mode == SCAN_BALL_DOWN) ? 0 : 1;

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

      case SCAN_MARATHON:
        {
          if (init_scanning()) {
            printf("head init\n");
            scan_left_limit = 70.0;
            scan_right_limit = -70.0;

            scan_top_limit = 0.0;
            scan_bottom_limit = -70.0;

            scan_speed = 0.20;

            scan_pan_angle = get_pan_angle();
            scan_tilt_angle = get_tilt_angle();

            scan_position = 0;
            marathon_index = -1;

            break;
          }

          switch (scan_position) {
            case 0:
              {
                marathon_index = scan_position;
                scan_tilt_angle += scan_speed;
                scan_pan_angle = 0;
                if (scan_tilt_angle > scan_top_limit) {
                  scan_position = 1;
                  scan_tilt_angle = bottom_limit;
                }

                break;
              }

            case 1:
              {
                marathon_index = scan_position;
                scan_tilt_angle += scan_speed;
                scan_pan_angle = scan_left_limit;
                if (scan_tilt_angle > scan_top_limit) {
                  scan_position = 2;
                  scan_tilt_angle = bottom_limit;
                }

                break;
              }

            case 2:
              {
                marathon_index = scan_position;
                scan_tilt_angle += scan_speed;
                scan_pan_angle = scan_right_limit;
                if (scan_tilt_angle > scan_top_limit) {
                  scan_position = 3;
                  scan_tilt_angle = bottom_limit;
                }

                break;
              }

            default:
              {
                marathon_index = scan_position;
                scan_pan_angle = 0.0;
                scan_tilt_angle = scan_bottom_limit;

                break;
              }
          }

          break;
        }
    }

    pan_angle = pan_center + alg::clampValue(scan_pan_angle, right_limit, left_limit);
    tilt_angle = tilt_center + alg::clampValue(scan_tilt_angle, bottom_limit, top_limit);
  }

  // if pan enable
  joints[0].set_target_position(pan_angle);

  // if tilt enable
  joints[1].set_target_position(tilt_angle);
}

void Head::move_scan(int mode)
{
  start();

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
  float dx = position_x - walking->POSITION_X;
  float dy = position_y - walking->POSITION_Y;

  float pan = imu->get_yaw() - (alg::direction(dx, dy) * alg::rad2Deg());
  float tilt = calculate_tilt_from_pan_distance(alg::distance(dx, dy));

  move_by_angle(pan - pan_center, tilt);
}

void Head::set_pan_tilt_angle(double pan, double tilt)
{
  current_pan_angle = pan;
  current_tilt_angle = tilt;
}

}  // namespace atama
