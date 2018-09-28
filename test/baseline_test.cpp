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

#include <jaco_manipulation/client/jaco_manipulation_client.h>
#include <jaco_manipulation/BoundingBox.h>
#include <boost/algorithm/string.hpp>
#include <jaco_manipulation/units.h>
#include <fstream>
#include <vector>

using std::string;
using std::ifstream;
using std::vector;
using namespace jaco_manipulation;

class CSVReader {
 public:
  CSVReader() = delete;
  CSVReader(const string filename, string delim=",") : delimiter(delim) {
    file = ifstream(filename);
    if (file)
      processFile(filename);

    data.reserve(100);
  }

  ~CSVReader() {
    file.close();
  }

  vector<BoundingBox> getData() const {
    return data;
  }

 private:
  void saveVecAsBoundingBox(const vector<string> &line) {
    assert(line.size() == 6);

    BoundingBox b;
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
  void processFile(const string &filename) {
    string line;
    size_t line_counter = 0;

    while (getline(file, line)) {
      ++line_counter;

      vector<string> vec;
      boost::algorithm::split(vec, line, boost::is_any_of(delimiter));
      if (line_counter == 1)
        continue;

      saveVecAsBoundingBox(vec);
    }
  }

  vector<BoundingBox> data;
  string delimiter;
  ifstream file;
};

int main(int argn, char *args[]) {

  ros::init(argn, args, "pam_client");

  CSVReader reader("/home/chitt/julian/reground_workspace/src/arm/jaco_manipulation/scripts/baseline_poses.csv");
  auto data = reader.getData();
  size_t test_num = 0;

  client::JacoManipulationClient jmc;

  for (const auto &box: data) {
    ROS_INFO_STREAM("\n---\nTest " << test_num++ << "\n---");
    jmc.graspAt(box);
    jmc.dropAt(box);
  }


  return 0;
}
