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

#include <string>

#include "atama/head/control/helper/parameter.hpp"

#include "nlohmann/json.hpp"

namespace atama::control
{

std::string Parameter::scan_custom(
  double left_limit, double right_limit,
  double top_limit, double bottom_limit)
{
  nlohmann::json param = {
    {"left_limit", left_limit},
    {"right_limit", right_limit},
    {"top_limit", top_limit},
    {"bottom_limit", bottom_limit},
  };

  return param.dump();
}

std::string Parameter::track_object(const std::string &object_name)
{
  nlohmann::json param = {
    {"object_name", object_name},
  };

  return param.dump();
}

std::string Parameter::move_by_angle(double pan_angle, double tilt_angle)
{
  nlohmann::json param = {
    {"pan_angle", pan_angle},
    {"tilt_angle", tilt_angle},
  };

  return param.dump();
}

std::string Parameter::look_to_position(double goal_position_x, double goal_position_y)
{
  nlohmann::json param = {
    {"goal_position_x", goal_position_x},
    {"goal_position_y", goal_position_y},
  };

  return param.dump();
}

} // namespace atama::control
