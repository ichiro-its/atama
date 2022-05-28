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

#ifndef ATAMA__HEAD__CONTROL__HELPER__PARAMETER_HPP_
#define ATAMA__HEAD__CONTROL__HELPER__PARAMETER_HPP_

#include <string>

namespace atama::control
{

class Parameter
{
public:
  static std::string scan_custom(
    double left_limit, double right_limit,
    double top_limit, double bottom_limit);

  static std::string track_object(const std::string &object_name);
  static std::string move_by_angle(double pan_angle, double tilt_angle);
  static std::string look_to_position(double goal_position_x, double goal_position_y);
};

} // namespace atama::control

#endif // ATAMA__HEAD__CONTROL__HELPER__PARAMETER_HPP_
