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

#ifndef PROJECT_CSV_READER_H
#define PROJECT_CSV_READER_H

#include <vector>
#include <fstream>
#include <string>
#include <jaco_manipulation/BoundingBox.h>

namespace jaco_manipulation {
namespace test {

class CSVReader {
 public:
  CSVReader() = delete;
  CSVReader(const std::string filename, std::string delim = ",");

  virtual ~CSVReader();

  std::vector<jaco_manipulation::BoundingBox> getData() const;

 private:
  void saveVec(const std::vector<std::string> &line);

  void processFile(const std::string &filename);

  std::vector<jaco_manipulation::BoundingBox> data;
  std::string delimiter;
  std::ifstream file;
};

} // namespace test
} // namespace jaco_manipulation

#endif //PROJECT_CSV_READER_H
