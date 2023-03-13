// Copyright (c) 2023 Ichiro ITS
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

#ifndef ATAMA__CONFIG__NODE__CONFIG_NODE_HPP_
#define ATAMA__CONFIG__NODE__CONFIG_NODE_HPP_

#include <memory>
#include <string>

#include "atama/config/utils/config.hpp"
#include "atama_interfaces/srv/get_config.hpp"
#include "atama_interfaces/srv/save_config.hpp"
#include "rclcpp/rclcpp.hpp"

namespace atama
{
class ConfigNode
{
public:
  ConfigNode(std::shared_ptr<rclcpp::Node> node, const std::string & path);

private:
  std::shared_ptr<rclcpp::Node> node;
  rclcpp::Service<atama_interfaces::srv::GetConfig>::SharedPtr get_config_service;
  rclcpp::Service<atama_interfaces::srv::SaveConfig>::SharedPtr set_config_service;
  Config config;
};
}  // namespace atama

#endif  // ATAMA__CONFIG__NODE__CONFIG_NODE_HPP_
