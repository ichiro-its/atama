#include "atama/head/process/head.hpp"
#include "atama/node/atama_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char const * argv[])
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
  atama_node->run_config_service(path);

  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
