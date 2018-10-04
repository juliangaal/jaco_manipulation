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
#include <jaco_manipulation/test/baseline_csv_reader.h>

using namespace jaco_manipulation;

int main(int argn, char *args[]) {

  ros::init(argn, args, "pam_client");

  test::BaselineCSVReader reader("/home/chitt/julian/reground_workspace/src/arm/jaco_manipulation/scripts/baseline_poses.csv");
  auto data = reader.getData();
  int test_num = 0;

  client::JacoManipulationClient jmc;

  for (const auto &box: data) {
    bool grip = test_num % 2 == 0 || test_num == 0;
    ROS_INFO_STREAM("\n---\nTest " << ++test_num << (grip ? ": Grip" : ": Drop") << "\n---");

    if (grip)
      jmc.graspAt(box);
    else
      jmc.dropAt(box);
  }


  return 0;
}
