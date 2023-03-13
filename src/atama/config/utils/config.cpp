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

#include "atama/config/utils/config.hpp"

#include <fstream>
#include <iomanip>
#include <string>

namespace atama
{
Config::Config(const std::string & path)
: path(path) {}

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
