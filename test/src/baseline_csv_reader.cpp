/*
  Copyright (C) 2018  Julian Gaal
  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.
  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.
  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>
*/

//#include "../include/jaco_manipulation/test/baseline_csv_reader.h"
#include <jaco_manipulation/test/baseline_csv_reader.h>
#include <boost/algorithm/string.hpp>


using namespace jaco_manipulation::test;

BaselineCSVReader::BaselineCSVReader(std::string filename, std::string delimiter) : CSVReader(filename, delimiter) {
  if (file)
    processFile(filename);
}

void BaselineCSVReader::saveVec(const std::vector<std::string> &line) {
  assert(line.size() == 6);

  jaco_manipulation::BoundingBox b;
  b.header.frame_id = "base_link";
  b.description = "box";
  b.point.x = std::stof(line[0]);
  b.point.y = std::stof(line[1]);
  b.point.z = std::stof(line[5]) - std::stof(line[5]) / 2;
  b.dimensions.x = std::stof(line[3]);
  b.dimensions.y = std::stof(line[4]);
  b.dimensions.z = std::stof(line[5]);
  data.emplace_back(std::move(b));
}

void BaselineCSVReader::processFile(const std::string &filename) {
  std::string line;
  size_t line_counter = 0;
  using namespace boost::algorithm;

  while (getline(file, line)) {
    ++line_counter;
    std::vector<std::string> vec;
    split(vec, line, boost::is_any_of(delimiter));
    if (line_counter == 1)
      continue;

    saveVec(vec);
  }
}