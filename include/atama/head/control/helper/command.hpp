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

#ifndef ATAMA__HEAD__CONTROL__HELPER__COMMAND_HPP_
#define ATAMA__HEAD__CONTROL__HELPER__COMMAND_HPP_

namespace atama::control
{

enum Command
{
  NONE              = -1,
  SCAN_UP           = 0,
  SCAN_DOWN         = 1,
  SCAN_VERTICAL     = 2,
  SCAN_HORIZONTAL   = 3,
  SCAN_MARATHON     = 4,
  SCAN_CUSTOM       = 5,
  TRACK_OBJECT      = 6,
  MOVE_BY_ANGLE     = 7,
  LOOK_TO_POSITION  = 8,
};

}  // namespace atama::control

#endif  // ATAMA__HEAD__CONTROL__HELPER__COMMAND_HPP_
