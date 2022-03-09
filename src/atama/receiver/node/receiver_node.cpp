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
#include <set>
#include <string>
#include <vector>

#include "atama/receiver/node/receiver_node.hpp"
#include "kansei_interfaces/msg/orientation.hpp"
#include "ninshiki_interfaces/msg/detected_object.hpp"
#include "ninshiki_interfaces/msg/detected_objects.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tachimawari_interfaces/srv/get_joints.hpp"

using namespace std::chrono_literals;

namespace atama::receiver
{

ReceiverNode::ReceiverNode(rclcpp::Node::SharedPtr node, std::shared_ptr<atama::head::Head> head)
: node(node), head(head)
{
  using ninshiki_interfaces::msg::DetectedObject;

  node_prefix = "receiver";

  get_joints_client = node->create_client<GetJoints>("/joint/get_joints");

  get_orientation_subsciber = node->create_subscription<Orientation>(
    "measurement/orientation", 10,
    [this](const Orientation::SharedPtr message) {
      // this->head->set_yaw(message->orientation[2]);
    }
  );

  get_detection_result_subsciber =
    node->create_subscription<DetectedObjects>(
    "ninshiki_py/detection", 10,
    [this](const DetectedObjects::SharedPtr message) {
      std::vector<DetectedObject> temp_detection_result;

      for (const auto & detected_object : message->detected_objects) {
        temp_detection_result.push_back(detected_object);
      }

      this->head->detection_result = temp_detection_result;
    }
    );

  get_camera_config_subsciber = node->create_subscription<CameraConfig>(
    "/camera/camera_config", 10,
    [this](const CameraConfig::SharedPtr message) {
      this->head->camera_width = message->width;
      this->head->camera_height = message->height;
      this->head->view_v_angle = message->v_angle;
      this->head->view_h_angle = message->h_angle;
    }
  );
}

bool ReceiverNode::get_joints_data()
{
  using tachimawari::joint::Joint;
  using tachimawari::joint::JointId;

  while (!get_joints_client->wait_for_service(1s)) {
    if (rclcpp::ok()) {
      // service not available, waiting again...
    } else {
      // Interrupted while waiting for the service. Exiting.
      return false;
    }
  }
  auto result = get_joints_client->async_send_request(
    std::make_shared<tachimawari_interfaces::srv::GetJoints::Request>());

  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    std::vector<Joint> temp_joints;
    std::set<uint8_t> joints_id;

    for (const std::string & id : {"neck_yaw", "neck_pitch"}) {
      joints_id.insert(JointId::by_name.find(id)->second);
    }

    for (const auto & joint : result.get()->joints) {
      // Joint Id found in joints_id set
      if (joints_id.find(joint.id) != joints_id.end()) {
        temp_joints.push_back(Joint(joint.id, joint.position));
      }
    }

    head->set_joints(temp_joints);
    return true;
  } else {
    // Failed to call service
    return false;
  }
}

}  // namespace atama::receiver
