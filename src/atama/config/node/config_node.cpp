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

#include "atama/config/node/config_node.hpp"

#include <fstream>
#include <memory>
#include <string>

#include "atama_interfaces/srv/get_config.hpp"
#include "atama_interfaces/srv/save_config.hpp"
namespace atama
{
ConfigNode::ConfigNode(std::shared_ptr<rclcpp::Node> node, const std::string & path)
: node(node), config(path)
{
  // get config
  get_config_service = node->create_service<atama_interfaces::srv::GetConfig>(
    "/atama_app/config/get_config",
    [this](
      atama_interfaces::srv::GetConfig::Request::SharedPtr request,
      atama_interfaces::srv::GetConfig::Response::SharedPtr response) {
      response->json = this->config.get_config();
    });

  set_config_service = node->create_service<atama_interfaces::srv::SaveConfig>(
    "/atama_app/config/save_config",
    [this](
      atama_interfaces::srv::SaveConfig::Request::SharedPtr request,
      atama_interfaces::srv::SaveConfig::Response::SharedPtr response) {
      try {
        nlohmann::json json = nlohmann::json::parse(request->json);

        this->config.set_config({json});
        response->status = true;
      } catch (...) {
        response->status = false;
      }
      ;
    });
}
}  // namespace atama
