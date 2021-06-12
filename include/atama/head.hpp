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

#ifndef ATAMA__HEAD_HPP_
#define ATAMA__HEAD_HPP_

#include <aruku/walking.hpp>
#include <kansei/imu.hpp>
#include <keisan/geometry/point_2.hpp>
#include <robocup_client/robocup_client.hpp>

#include <memory>
#include <string>
#include <vector>

namespace atama
{

class Head
{
public:
  enum
  {
    SCAN_BALL_UP                = 0,
    SCAN_BALL_DOWN              = 1,
    SCAN_VERTICAL               = 2,
    SCAN_HORIZONTAL             = 3,
    SCAN_MARATHON               = 4
  };

  int marathon_index;

  Head(std::shared_ptr<aruku::Walking> walking, std::shared_ptr<kansei::Imu> imu);

  void start_scan() {is_started_scanning = true;}
  void stop_scan() {is_started_scanning = false; scan_init = false;}

  void initialize();
  void process();
  void init_tracking();

  // void joint_enable() { m_Joint.SetEnableHead_only(true, true); }
  // void joint_disable() { m_Joint.SetEnableBody(false); }

  double get_top_limit_angle() {return top_limit;}
  double get_bottom_limit_angle() {return bottom_limit;}
  double get_right_limit_angle() {return right_limit;}
  double get_left_limit_angle() {return left_limit;}

  double get_pan_angle() {return pan_angle - pan_center;}
  double get_tilt_angle() {return tilt_angle - tilt_center;}

  double get_pan_error() {return pan_error;}
  double get_tilt_error() {return tilt_error;}

  double get_left_limit() {return left_limit;}
  double get_right_limit() {return right_limit;}
  double get_top_limit() {return top_limit;}
  double get_bottom_limit() {return bottom_limit;}

  double get_pan_center() {return pan_center;}
  double get_tilt_center() {return tilt_center;}

  void move_by_angle(double pan_angle, double tilt_angle);

  void move_tracking(double pan, double tilt);
  void move_tracking_pan_only(double pan);
  void move_tracking_tilt_only(double tilt);
  void move_tracking();

  void reinit_scan() {scan_init = false;}

  void move_scan(int mode);
  void move_scan_ball_up() {move_scan(SCAN_BALL_UP);}
  void move_scan_ball_down() {move_scan(SCAN_BALL_DOWN);}
  void move_scan_horizontal() {move_scan(SCAN_HORIZONTAL);}
  void move_scan_vertical() {move_scan(SCAN_VERTICAL);}
  void move_scan_marathon() {move_scan(SCAN_MARATHON);}

  double calculate_distance_from_pan_tilt()
  {
    return calculate_distance_from_pan_tilt(get_pan_angle(), get_tilt_angle());
  }
  double calculate_distance_from_pan_tilt(double pan, double tilt);
  double calculate_tilt_from_pan_distance(double distance)
  {
    return calculate_tilt_from_pan_distance(get_pan_angle(), distance);
  }
  double calculate_tilt_from_pan_distance(double pan, double distance);

  void look_to_position(double position_x, double position_y);

  void load_data(std::string file_name);

  void track_ball(
    std::shared_ptr<Head> head, std::shared_ptr<CameraMeasurement> camera,
    keisan::Point2 pos, float view_v_angle, float view_h_angle);

  // REQUIRED
  void set_pan_tilt_angle(double pan, double tilt);

  std::vector<tachimawari::Joint> get_joints() {return joints;}

private:
  bool init_scanning();

  bool is_started_scanning;
  bool pan_only;
  bool tilt_only;

  double pan_angle;
  double tilt_angle;

  double pan_center;
  double tilt_center;

  double pan_offset;
  double tilt_offset;
  double offset_x;
  double offset_y;

  double left_limit;
  double right_limit;
  double top_limit;
  double bottom_limit;

  double pan_p_gain;
  double pan_d_gain;
  double tilt_p_gain;
  double tilt_d_gain;

  double pan_tilt_to_distance_[7][7];
  double pan_distance_to_tilt_[5][5];

  double pan_error;
  double pan_error_difference;

  double tilt_error;
  double tilt_error_difference;

  int scan_mode;

  bool scan_init;

  double scan_pan_angle;
  double scan_tilt_angle;

  double scan_left_limit;
  double scan_right_limit;
  double scan_top_limit;
  double scan_bottom_limit;

  double scan_speed;

  int scan_position;
  int scan_direction;

  double current_pan_angle;
  double current_tilt_angle;

  int no_ball_count;
  int ball_count;
  static const int no_ball_max_count = 1;
  static const int ball_max_count = 8;

  keisan::Point2 ball_position;

  std::vector<tachimawari::Joint> joints;

  std::shared_ptr<kansei::Imu> imu;
  std::shared_ptr<aruku::Walking> walking;
};

}  // namespace atama

#endif  // ATAMA__HEAD_HPP_
