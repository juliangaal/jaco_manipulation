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

#ifndef PROJECT_BASELINE_CSV_READER_H
#define PROJECT_BASELINE_CSV_READER_H

#include <jaco_manipulation/test/csv_reader.h>

namespace jaco_manipulation {
namespace test {

class BaselineCSVReader : public CSVReader {
 public:
  BaselineCSVReader() = delete;
  explicit BaselineCSVReader(std::string filename, std::string delimiter=",");
  ~BaselineCSVReader() final = default;
 private:
  void saveVec(const std::vector<std::string> &line) override;
  void processFile(const std::string &filename) override;
};

} // namespace test
} // namespace jaco_manipulation

#endif //PROJECT_BASELINE_CSV_READER_H
