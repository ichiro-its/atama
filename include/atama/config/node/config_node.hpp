#ifndef ATAMA__CONFIG__NODE__CONFIG_NODE_HPP_
#define ATAMA__CONFIG__NODE__CONFIG_NODE_HPP_

#include "atama/config/utils/config.hpp"
#include "atama_interfaces/srv/get_config.hpp"
#include "atama_interfaces/srv/save_config.hpp""
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