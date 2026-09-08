#pragma once

#include <string>
#include <vector>
#include <iostream>

#include <boost/filesystem.hpp>
#include <boost/range/iterator_range.hpp>

namespace utils {

std::string PathJoin(const std::string &base_path, const std::string &name);

std::vector<std::string> ListFiles(const std::string &directory_name);



}