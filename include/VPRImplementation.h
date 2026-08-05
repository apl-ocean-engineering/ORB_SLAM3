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

#include <memory>
#include <vector>

#include "Frame.h"
#include "KeyFrame.h"
#include "Map.h"
#include "ORBVocabulary.h"

namespace ORB_SLAM3 {

class Map;

class VPRImplementation {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  VPRImplementation() = default;

  virtual void add(const std::shared_ptr<KeyFrame> &pKF) = 0;
  virtual void erase(const std::shared_ptr<KeyFrame> &pKF) = 0;

  virtual void clear() = 0;
  virtual void clearMap(const std::shared_ptr<Map> &pMap) = 0;

  // Loop and Merge Detection
  // These aren't used?
  //
  //   virtual void DetectCandidates(
  //       const std::shared_ptr<KeyFrame> &pKF, float minScore,
  //       vector<std::shared_ptr<KeyFrame>> &vpLoopCand,
  //       vector<std::shared_ptr<KeyFrame>> &vpMergeCand) = 0;
  //   virtual void DetectBestCandidates(
  //       const std::shared_ptr<KeyFrame> &pKF,
  //       vector<std::shared_ptr<KeyFrame>> &vpLoopCand,
  //       vector<std::shared_ptr<KeyFrame>> &vpMergeCand, int nMinWords) = 0;

  virtual void DetectNBestCandidates(
      const std::shared_ptr<KeyFrame> &pKF,
      vector<std::shared_ptr<KeyFrame>> &vpLoopCand,
      vector<std::shared_ptr<KeyFrame>> &vpMergeCand,
      size_t nNumCandidates) = 0;

  // Relocalization
  virtual std::vector<std::shared_ptr<KeyFrame>> DetectRelocalizationCandidates(
      const std::shared_ptr<Frame> &F, const std::shared_ptr<Map> &pMap) = 0;

 protected:
};

}  // namespace ORB_SLAM3
