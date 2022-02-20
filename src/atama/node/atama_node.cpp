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

#include <chrono>
#include <memory>
#include <string>

#include "atama/node/atama_node.hpp"

using namespace std::chrono_literals;

namespace atama
{

AtamaNode::AtamaNode(rclcpp::Node::SharedPtr node)
: node(node), receiver_node(nullptr), sender_node(nullptr)
{
  node_timer = node->create_wall_timer(
    8ms,
    [this]() {
      if (receiver_node != nullptr) {
        receiver_node->get_joints_data();
        
        if (sender_node != nullptr) {
          sender_node->publish_joints();
        }
      }
    }
  );
}

void AtamaNode::set_receiver_node(std::shared_ptr<atama::head::Head> head)
{
  receiver_node = std::make_shared<atama::receiver::ReceiverNode>(
    node, head);
}

void AtamaNode::set_sender_node(std::shared_ptr<atama::head::Head> head)
{
  sender_node = std::make_shared<atama::sender::SenderNode>(
    node, head);
}

}  // namespace atama