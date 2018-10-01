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

#include <jaco_manipulation/test/csv_reader.h>
#include <ros/console.h>

using namespace jaco_manipulation::test;

CSVReader::CSVReader(const std::string filename, std::string delim) : delimiter(delim) {
  file = std::ifstream(filename);
  if (file) {
    data.reserve(100);
  } else {
    ROS_ERROR_STREAM("Can't open file " << filename << ". Exiting.");
  }
}

CSVReader::~CSVReader() {
  if (file)
    file.close();
}

const std::vector<jaco_manipulation::BoundingBox>& CSVReader::getData() const {
  return data;
}
