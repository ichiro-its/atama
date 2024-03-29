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

#include <memory>
#include <string>

#include "atama/head/process/head.hpp"
#include "atama/node/atama_node.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  if (argc < 2) {
    std::cerr << "Please specify the path!" << std::endl;
    return 0;
  }

  auto head = std::make_shared<atama::Head>();
  std::string path = argv[1];
  head->load_config(path);

  auto node = std::make_shared<rclcpp::Node>("atama_node");
  auto atama_node = std::make_shared<atama::AtamaNode>(node);
  atama_node->run_head_service(head);

  atama::HeadNode head_node(node, head);

  rclcpp::Rate rcl_rate(8ms);
  while (rclcpp::ok()) {
    rcl_rate.sleep();

    rclcpp::spin_some(node);

    head->scan_custom(atama::control::Command::SCAN_UP);
    head_node.update();
  }

  rclcpp::shutdown();

  return 0;
}
