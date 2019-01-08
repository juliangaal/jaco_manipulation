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

#ifndef PROJECT_UNITS_H
#define PROJECT_UNITS_H

namespace jaco_manipulation {

/**
 * User defined literal: Meters
 * @param amount 
 * @return amount in meters
 */
constexpr double operator "" _m(long double amount) {
  return static_cast<double>(amount);
}

/**
 * User defined literal: Centimeters
 * @param amount
 * @return amount in cms
 */
constexpr double operator "" _cm(long double amount) {
  return static_cast<double>(amount / 100.0);
}

/**
 * User defined literal: Millimeters
 * @param amount
 * @return amount in millimeters
 */
constexpr double operator "" _mm(long double amount) {
  return static_cast<double>(amount / 1000.0);
}

/**
 * User defined literal: Degree
 * @param amount
 * @return amount in degree (may be used to convert to rad in the future
 */
constexpr double operator "" _deg(long double amount) {
  return static_cast<double>(amount);
}

/**
 * User defined literal: Percent
 * @param amount
 * @return amount in percent
 */
constexpr double operator "" _percent(long double amount) {
  return static_cast<double>(amount);
}

} // namespace jaco_manipulation

#endif //PROJECT_UNITS_H
