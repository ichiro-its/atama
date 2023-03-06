#include "atama/config/node/config_node.hpp"

#include "atama_interfaces/srv/get_config.hpp"
#include "atama_interfaces/srv/save_config.hpp"

namespace atama
{
ConfigNode::ConfigNode(std::shared_ptr<rclcpp::Node> node, const std::string & path)
: node(node), config(path)
{
  using namespace atama_interfaces::srv;
  // get config
  get_config_service = node->create_service<GetConfig>(
    "/atama_app/config/get_config",
    [this](GetConfig::Request::SharedPtr request, GetConfig::Response::SharedPtr response) {
      response->json_data = this->config.get_config();
    });

  set_config_service = node->create_service<SaveConfig>(
    "/atama_app/config/set_config",
    [this](SaveConfig::Request::SharedPtr request, SaveConfig::Response::SharedPtr response) {
      this->config.set_config(request->json_data);
    });
}
}  // namespace atama