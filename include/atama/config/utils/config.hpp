#ifndef ATAMA__CONFIG__UTILS__CONFIG_HPP_
#define ATAMA__CONFIG__UTILS__CONFIG_HPP_

#include <string>

#include "nlohmann/json.hpp"

namespace atama
{
class Config
{
public:
  Config(const std::string & path);
  std::string get_config();
  void set_config(const nlohmann::json & config);

private:
  const std::string & path;
};
}  // namespace atama

#endif