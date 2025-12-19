/**
 * This file is part of ORB-SLAM3
 *
 * Copyright (C) 2017-2021 Carlos Campos, Richard Elvira, Juan J. Gómez
 * Rodríguez, José M.M. Montiel and Juan D. Tardós, University of Zaragoza.
 * Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel and Juan D. Tardós,
 * University of Zaragoza.
 *
 * ORB-SLAM3 is free software: you can redistribute it and/or modify it under
 * the terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * ORB-SLAM3 is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE. See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * ORB-SLAM3. If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace ORB_SLAM3 {

class Verbose {
 public:
  enum eLevel {
    VERBOSITY_QUIET = 0,
    VERBOSITY_NORMAL = 1,
    VERBOSITY_VERBOSE = 2,
    VERBOSITY_VERY_VERBOSE = 3,
    VERBOSITY_DEBUG = 4
  };

  static eLevel th;

 public:
  static void PrintMess(std::string str, eLevel lev) {
    switch (lev) {
      case VERBOSITY_DEBUG:
        spdlog::debug("{}", str);
        break;
      case VERBOSITY_VERY_VERBOSE:
        spdlog::debug("{}", str);
        break;
      case VERBOSITY_VERBOSE:
        spdlog::info("{}", str);
        break;
      case VERBOSITY_NORMAL:
        spdlog::error("{}", str);
        break;
      case VERBOSITY_QUIET:
        spdlog::critical("{}", str);
        break;
    }
  }

  static void SetTh(eLevel _th) {}

  static void set_default_logger(
      const std::shared_ptr<spdlog::logger>& logger) {
    spdlog::set_default_logger(logger);
  }
};

}  // namespace ORB_SLAM3
