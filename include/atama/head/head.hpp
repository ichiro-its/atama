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

#ifndef ATAMA__HEAD__HEAD_HPP_
#define ATAMA__HEAD__HEAD_HPP_

#include <time.h>

#include <memory>
#include <string>
#include <vector>

#include "keisan/geometry/point_2.hpp"
#include "ninshiki_interfaces/msg/detected_object.hpp"
#include "ninshiki_interfaces/msg/detected_objects.hpp"
#include "tachimawari/joint/model/joint.hpp"
#include "./common.h"

namespace atama::head
{

class Head
{
public:
  enum
  {
    SCAN_UP                = 0,
    SCAN_DOWN              = 1,
    SCAN_VERTICAL          = 2,
    SCAN_HORIZONTAL        = 3,
    SCAN_MARATHON          = 4,
    SCAN_CUSTOM            = 5,
    TRACK_OBJECT           = 6,
    MOVE_BY_ANGLE          = 7,
    LOOK_TO_POSITION       = 8,
  };
  std::vector<ninshiki_interfaces::msg::DetectedObject> detection_result;

  int marathon_index;

  Head(int camera_width, int camera_height);

  void start_scan() {is_started_scanning = true;}
  void stop_scan() {is_started_scanning = false; scan_init = false;}

  void initialize();

  // void joint_enable() { m_Joint.SetEnableHead_only(true, true); }
  // void joint_disable() { m_Joint.SetEnableBody(false); }

  double get_pan_angle() {return pan_angle - pan_center;}
  double get_tilt_angle() {return tilt_angle - tilt_center;}

  double get_pan_angle_goal() {return pan_angle_goal;}
  double get_tilt_angle_goal() {return tilt_angle_goal;}

  double get_pan_error() {return pan_error;}
  double get_tilt_error() {return tilt_error;}

  double get_left_limit() {return left_limit;}
  double get_right_limit() {return right_limit;}
  double get_top_limit() {return top_limit;}
  double get_bottom_limit() {return bottom_limit;}

  double get_scan_left_limit() {return scan_left_limit;}
  double get_scan_right_limit() {return scan_right_limit;}
  double get_scan_top_limit() {return scan_top_limit;}
  double get_scan_bottom_limit() {return scan_bottom_limit;}

  double get_pan_center() {return pan_center;}
  double get_tilt_center() {return tilt_center;}
  std::string get_object_name() {return object_name;}
  double get_goal_position_x() {return goal_position_x;}
  double get_goal_position_y() {return goal_position_y;}
  int get_function_id() {return function_id;}

  void set_scan_limit(
    double left_limit, double right_limit,
    double top_limit, double bottom_limit
  );

  void set_object_name(std::string value) {object_name = value;}
  void set_pan_angle_goal(double value) {pan_angle_goal = value;}
  void set_tilt_angle_goal(double value) {tilt_angle_goal = value;}
  void set_goal_position_x(double value) {goal_position_x = value;}
  void set_goal_position_y(double value) {goal_position_y = value;}
  void set_function_id(int value) {function_id = value;}

  void move_by_angle(double pan_angle, double tilt_angle);

  void tracking(double pan, double tilt);
  void tracking_pan_only(double pan);
  void tracking_tilt_only(double tilt);
  void tracking();

  void reinit_scan() {scan_init = false;}

  void scan(int mode);
  void scan_up() {set_scan_limit(60.0, -60.0, 0.0, -75.0); scan_custom(SCAN_UP);}
  void scan_down() {set_scan_limit(60.0, -60.0, 0.0, -75.0); scan_custom(SCAN_DOWN);}
  void scan_horizontal() {set_scan_limit(70.0, -70.0, -30.0, -30.0); scan_custom(SCAN_HORIZONTAL);}
  void scan_vertical() {set_scan_limit(0.0, 0.0, 0.0, -70.0); scan_custom(SCAN_VERTICAL);}
  void scan_marathon() {set_scan_limit(70.0, -70.0, 0.0, -70.0); scan_custom(SCAN_MARATHON);}
  void scan_custom(int scan_type = SCAN_CUSTOM);

  double calculate_distance_from_pan_tilt()
  {
    return calculate_distance_from_pan_tilt(get_pan_angle(), get_tilt_angle());
  }
  double calculate_distance_from_pan_tilt(double pan, double tilt);

  double calculate_distance_from_tilt()
  {
    return calculate_distance_from_tilt(get_tilt_angle());
  }
  double calculate_distance_from_tilt(double tilt);

  double calculate_tilt_from_pan_distance(double distance)
  {
    return calculate_tilt_from_pan_distance(get_pan_angle(), distance);
  }
  double calculate_tilt_from_pan_distance(double pan, double distance);

  void look_to_position(
    double goal_position_x, double goal_position_y,
    double robot_position_x, double robot_position_y,
    float yaw);

  void load_data(std::string file_name);

  void track_object(std::string object_name);

  // REQUIRED
  void set_pan_tilt_angle(double pan, double tilt);

  void set_joints(std::vector<tachimawari::joint::Joint> joints) {joints = joints;}
  std::vector<tachimawari::joint::Joint> get_joints() {return joints;}
  bool is_joint_empty() {return joints.empty();}

  void set_detection_result(std::vector<ninshiki_interfaces::msg::DetectedObject> detection_result)
    {detection_result = detection_result;}
  std::vector<ninshiki_interfaces::msg::DetectedObject> get_detection_result()
    {return detection_result;}

private:
  bool init_scanning();
  void scan_process();
  bool check_time_belief();

  bool pan_only;
  bool tilt_only;

  double pan_angle;
  double tilt_angle;

  double pan_angle_goal;
  double tilt_angle_goal;

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
  double tilt_to_distance_[5];
  double pan_distance_to_tilt_[5][5];

  double pan_error;
  double pan_error_difference;

  double tilt_error;
  double tilt_error_difference;

  bool is_started_scanning;
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

  std::string object_name = "ball";
  int no_object_count;
  int object_count;
  static const int no_object_max_count = 1;
  static const int object_max_count = 8;

  std::vector<tachimawari::joint::Joint> joints;

  int camera_width;
  int camera_height;

  double goal_position_x;
  double goal_position_y;

  int function_id;
  clock_t min_time;
};

}  // namespace atama::head

#endif  // ATAMA__HEAD__HEAD_HPP_
