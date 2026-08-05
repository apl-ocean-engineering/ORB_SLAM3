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

#include "SystemAccessor.h"

#include <cassert>
#include <memory>

#include "Atlas.h"
#include "FrameDrawer.h"
#include "GeometricCamera.h"
#include "LocalMapping.h"
#include "LoopClosing.h"
#include "Settings.h"
#include "System.h"
#include "Tracking.h"

namespace ORB_SLAM3 {

SystemAccessor::SystemAccessor(const std::shared_ptr<System> &pSys)
    : mpSystem(pSys) {}

void SystemAccessor::setSystem(const std::shared_ptr<System> &pSys) {
  mpSystem = pSys;
}

std::shared_ptr<System> SystemAccessor::system() {
  assert(mpSystem);
  return mpSystem;
}

const std::shared_ptr<System> SystemAccessor::system() const {
  assert(mpSystem);
  return mpSystem;
}

const SensorType SystemAccessor::sensorType() const {
  return system()->sensorType();
}

std::shared_ptr<Settings> SystemAccessor::getSettings() {
  return system()->getSettings();
}

std::shared_ptr<Tracking> SystemAccessor::getTracker() {
  return system()->getTracker();
}

std::shared_ptr<Viewer> SystemAccessor::getViewer() {
  return system()->getViewer();
}
std::shared_ptr<FrameDrawer> SystemAccessor::getFrameDrawer() {
  return system()->getFrameDrawer();
}
std::shared_ptr<MapDrawer> SystemAccessor::getMapDrawer() {
  return system()->getMapDrawer();
}

std::shared_ptr<Atlas> SystemAccessor::getAtlas() {
  return system()->getAtlas();
}

std::shared_ptr<LocalMapping> SystemAccessor::getLocalMapper() {
  return system()->getLocalMapping();
}
std::shared_ptr<LoopClosing> SystemAccessor::getLoopClosing() {
  return system()->getLoopClosing();
}

std::shared_ptr<VPRImplementation> SystemAccessor::getVPRImplementation() {
  return system()->getVPRImplementation();
}

std::shared_ptr<GeometricCamera> SystemAccessor::getPrimaryCamera() {
  return system()->getPrimaryCamera();
}

std::shared_ptr<GeometricCamera> SystemAccessor::getSecondaryCamera() {
  return system()->getSecondaryCamera();
}

}  // namespace ORB_SLAM3
