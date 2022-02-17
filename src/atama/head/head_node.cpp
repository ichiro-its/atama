// Copyright (c) 2021 Ichiro ITS
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#include <memory>
#include <string>
#include <vector>

#include "atama/head/head_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tachimawari_interfaces/msg/set_joints.hpp"
#include "tachimawari_interfaces/srv/get_joints.hpp"

using namespace std::chrono_literals;

namespace atama
{

HeadNode::HeadNode(rclcpp::Node::SharedPtr node, std::shared_ptr<Head> head)
: node(node), head(head)
{
  set_joints_publisher = node->create_publisher<tachimawari_interfaces::msg::SetJoints>(
    "/joint/set_joints", 10);

  get_joints_client = node->create_client<tachimawari_interfaces::srv::GetJoints>(
    "/joint/get_joints");
}

void HeadNode::get_joints_data()
{
  std::cout << "get_joints_data" << std::endl;
  while (!get_joints_client->wait_for_service(1s)) {
    if (rclcpp::ok()) {
      // service not available, waiting again...
    } else {
      // Interrupted while waiting for the service. Exiting.
      return;
    }
  }
  std::cout << "get_joints_data" << std::endl;

  auto result = get_joints_client->async_send_request(
    std::make_shared<tachimawari_interfaces::srv::GetJoints::Request>());

  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    std::vector<tachimawari::joint::Joint> temp_joints;

    for (const auto & joint : result.get()->joints) {
      for (std::string id : {"neck_yaw", "neck_pitch"}) {
        uint8_t joint_id = tachimawari::joint::JointId::by_name.find(id)->second;

        if (joint.id == joint_id) {
          temp_joints.push_back(
            tachimawari::joint::Joint(joint.id, joint.position));
        }
      }
    }

    head->set_joints(temp_joints);
  } else {
    // Failed to call service
    return;
  }
}

void HeadNode::publish_joints()
{
  std::cout << "publish_joints" << std::endl;
  auto joints_msg = tachimawari_interfaces::msg::SetJoints();

  const auto & joints = head->get_joints();
  auto & joint_msgs = joints_msg.joints;

  joint_msgs.resize(joints.size());
  for (size_t i = 0; i < joints.size() && i < joint_msgs.size(); ++i) {
    joint_msgs[i].id = joints[i].get_id();
    joint_msgs[i].position = joints[i].get_position();
  }

  set_joints_publisher->publish(joints_msg);
}

std::string HeadNode::get_node_prefix() const
{
  return "head";
}

}  // namespace atama
