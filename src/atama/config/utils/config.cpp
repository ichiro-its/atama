#include "atama/config/utils/config.hpp"

#include <fstream>
#include <iomanip>
#include <string>

namespace atama
{
Config::Config(const std::string & path) : path(path) {}

std::string Config::get_config()
{
  std::ifstream config_file(path + "/head.json");
  nlohmann::json config_data = nlohmann::json::parse(config_file);
  config_file.close();
  return config_data.dump();
}

void Config::set_config(const nlohmann::json & config)
{
  std::ofstream config_file(path + "/head.json", std::ios::out | std::ios::trunc);
  config_file << std::setw(2) << config << std::endl;
  config_file.close();
}

}  // namespace atama