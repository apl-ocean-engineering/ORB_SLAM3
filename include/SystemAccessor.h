/**
 * This file is part of ORB-SLAM3
 *
 * Copytight (c) 2026 University of Washington
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

#include <memory>

namespace ORB_SLAM3 {

class Atlas;
class FrameDrawer;
class LocalMapping;
class LoopClosing;
class MapDrawer;
class Settings;
class System;
class Tracking;
class Viewer;

class SystemAccessor {
 public:
  explicit SystemAccessor(const std::shared_ptr<System> &pSys = nullptr);

  // For setting System after construction (when restoring using boost::archive)
  void setSystem(const std::shared_ptr<System> &pSys);

 protected:
  // System
  std::shared_ptr<System> system();

  std::shared_ptr<Settings> getSettings();
  std::shared_ptr<LocalMapping> getLocalMapper();
  std::shared_ptr<LoopClosing> getLoopClosing();
  std::shared_ptr<Tracking> getTracker();

  // Drawers
  std::shared_ptr<Viewer> getViewer();

  std::shared_ptr<FrameDrawer> getFrameDrawer();
  std::shared_ptr<MapDrawer> getMapDrawer();

  // Atlas
  std::shared_ptr<Atlas> getAtlas();

 private:
  // System
  std::shared_ptr<System> mpSystem;
};

}  // namespace ORB_SLAM3
