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

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>

#include <memory>
#include <opencv2/core/core.hpp>
#include <string>
#include <thread>
#include <vector>

#include "Atlas.h"
#include "Expected.h"
#include "FrameDrawer.h"
#include "ImuTypes.h"
#include "KeyFrameDatabase.h"
#include "LocalMapping.h"
#include "Logging.h"
#include "LoopClosing.h"
#include "MapDrawer.h"
#include "ORBVocabulary.h"
#include "Settings.h"
#include "Tracking.h"
#include "Utils/FpsEstimator.h"
#include "Viewer.h"

namespace ORB_SLAM3 {

class Viewer;
class FrameDrawer;
class MapDrawer;
class Atlas;
class Tracking;
class LocalMapping;
class LoopClosing;
class Settings;

class SystemFactory {
 public:
  typedef tl::expected<std::shared_ptr<System>, ExpectedError> Expected;

  static Expected create(const std::shared_ptr<Settings> &settings,
                         bool initFr = false,
                         const string &strSequence = std::string());

  static Expected create(const std::string &configFile, const SensorType sensor,
                         bool initFr = false,
                         const string &strSequence = std::string());

  // Provided for compatibility with old API
  static Expected create(const std::string &configFile,
                         const std::string &vocabFile, const SensorType sensor,
                         bool initFr = false,
                         const string &strSequence = std::string());
};

// System should be created using  SystemFactory::create()
//
// It will validate settings and catch errors on startup
class System : public std::enable_shared_from_this<System> {
 public:
  friend SystemFactory::Expected SystemFactory::create(
      const std::shared_ptr<Settings> &, bool, const string &);

  // File type
  enum FileType {
    TEXT_FILE = 0,
    BINARY_FILE = 1,
  };

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // Proccess the given stereo frame. Images must be synchronized and rectified.
  // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to
  // grayscale. Returns the camera pose (empty if tracking fails).
  Sophus::SE3f TrackStereo(
      const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp,
      const vector<IMU::Point> &vImuMeas = vector<IMU::Point>(),
      string filename = "");

  // Process the given rgbd frame. Depthmap must be registered to the RGB frame.
  // Input image: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to
  // grayscale. Input depthmap: Float (CV_32F). Returns the camera pose (empty
  // if tracking fails).
  Sophus::SE3f TrackRGBD(
      const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp,
      const vector<IMU::Point> &vImuMeas = vector<IMU::Point>(),
      string filename = "");

  // Proccess the given monocular frame and optionally imu data
  // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to
  // grayscale. Returns the camera pose (empty if tracking fails).
  Sophus::SE3f TrackMonocular(
      const cv::Mat &im, const double &timestamp,
      const vector<IMU::Point> &vImuMeas = vector<IMU::Point>(),
      string filename = "");

  // This stops local mapping thread (map building) and performs only camera
  // tracking.
  void ActivateLocalizationMode();
  // This resumes local mapping thread and performs SLAM again.
  void DeactivateLocalizationMode();

  // Returns true if there have been a big map change (loop closure, global BA)
  // since last call to this function
  bool MapChanged();

  // Reset the system (clear Atlas or the active map)
  void Reset();
  void ResetActiveMap();

  // All threads will be requested to finish.
  // It waits until all threads have finished.
  // This function must be called before saving the trajectory.
  void Shutdown();
  bool isShutDown();

  // Save camera trajectory in the TUM RGB-D dataset format.
  // Only for stereo and RGB-D. This method does not work for monocular.
  // Call first Shutdown()
  // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
  void SaveTrajectoryTUM(const string &filename);

  // Save keyframe poses in the TUM RGB-D dataset format.
  // This method works for all sensor input.
  // Call first Shutdown()
  // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
  void SaveKeyFrameTrajectoryTUM(const string &filename);

  void SaveTrajectoryEuRoC(const string &filename);
  void SaveKeyFrameTrajectoryEuRoC(const string &filename);

  void SaveTrajectoryEuRoC(const string &filename,
                           const std::shared_ptr<Map> &pMap);
  void SaveKeyFrameTrajectoryEuRoC(const string &filename,
                                   const std::shared_ptr<Map> &pMap);

  // Save data used for initialization debug
  void SaveDebugData(const int &iniIdx);

  // Save camera trajectory in the KITTI dataset format.
  // Only for stereo and RGB-D. This method does not work for monocular.
  // Call first Shutdown()
  // See format details at:
  // http://www.cvlibs.net/datasets/kitti/eval_odometry.php
  void SaveTrajectoryKITTI(const string &filename);

  // \todo{} Serialization is currently broken
  // SaveMap(const string &filename);
  // LoadMap(const string &filename);

  // Information from most recent processed frame
  // You can call this right after TrackMonocular (or stereo or RGBD)
  int GetTrackingState();
  std::vector<MapPoint *> GetTrackedMapPoints();
  std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

  std::shared_ptr<FrameDrawer> frameDrawer() { return mpFrameDrawer; }
  std::shared_ptr<MapDrawer> mapDrawer() { return mpMapDrawer; }

  const SensorType sensorType() const { return settings_->sensor_; }

  // For debugging
  double GetTimeFromIMUInit();
  bool isLost();
  bool isFinished();

  float fps() const { return fps_estimator_.fps(); }

  void ChangeDataset();

  float GetImageScale();

#ifdef REGISTER_TIMES
  void InsertRectTime(double &time);
  void InsertResizeTime(double &time);
  void InsertTrackTime(double &time);
#endif

  // Member accessors
  std::shared_ptr<Atlas> getAtlas() { return mpAtlas; }

  std::shared_ptr<VPRImplementation> getVprImplementation() {
    return mpVprImpl;
  }
  std::shared_ptr<ORBVocabulary> getORBVocabulary() { return mpVocabulary; }

  std::shared_ptr<Settings> getSettings() { return settings_; }

  std::shared_ptr<Tracking> getTracker() { return mpTracker; }

  std::shared_ptr<Viewer> getViewer() { return mpViewer; }
  std::shared_ptr<FrameDrawer> getFrameDrawer() { return mpFrameDrawer; }
  std::shared_ptr<MapDrawer> getMapDrawer() { return mpMapDrawer; }

  std::shared_ptr<LocalMapping> getLocalMapping() { return mpLocalMapper; }
  std::shared_ptr<LoopClosing> getLoopClosing() { return mpLoopCloser; }

 protected:
  // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and
  // Viewer threads.
  //
  // All construction should go through the factory to ensure correct
  // initialization
  System(const std::shared_ptr<Settings> &settings, bool initFr = false,
         const string &strSequence = std::string());

  void printBanner();

  bool initialize(bool initFr = false,
                  const string &strSequence = std::string());

 private:
  void processLocalizationModeChange(void);
  void processReset(void);
  void updateTrackingState();

  void SaveAtlas(int type);
  bool LoadAtlas(int type);

  string CalculateCheckSum(string filename, int type);

  // ORB vocabulary used for place recognition and feature matching.
  std::shared_ptr<ORBVocabulary> mpVocabulary;

  // KeyFrame database for place recognition (relocalization and loop
  // detection).
  std::shared_ptr<VPRImplementation> mpVprImpl;

  // Map structure that stores the pointers to all KeyFrames and MapPoints.
  // Map* mpMap;
  std::shared_ptr<Atlas> mpAtlas;

  // Tracker. It receives a frame and computes the associated camera pose.
  // It also decides when to insert a new keyframe, create some new MapPoints
  // and performs relocalization if tracking fails.
  std::shared_ptr<Tracking> mpTracker;

  // Local Mapper. It manages the local map and performs local bundle
  // adjustment.
  std::shared_ptr<LocalMapping> mpLocalMapper;

  // Loop Closer. It searches loops with every new keyframe. If there is a loop
  // it performs a pose graph optimization and full bundle adjustment (in a new
  // thread) afterwards.
  std::shared_ptr<LoopClosing> mpLoopCloser;

  // The viewer draws the map and the current camera pose. It uses Pangolin.
  std::shared_ptr<Viewer> mpViewer;
  std::shared_ptr<FrameDrawer> mpFrameDrawer;
  std::shared_ptr<MapDrawer> mpMapDrawer;

  // System threads: Local Mapping, Loop Closing, Viewer.
  // The Tracking thread "lives" in the main execution thread that creates the
  // System object.
  std::unique_ptr<std::thread> mptLocalMapping;
  std::unique_ptr<std::thread> mptLoopClosing;
  std::unique_ptr<std::thread> mptViewer;

  // Reset flag
  std::mutex mMutexReset;
  bool mbReset;
  bool mbResetActiveMap;

  // Change mode flags
  std::mutex mMutexMode;
  bool mbActivateLocalizationMode;
  bool mbDeactivateLocalizationMode;

  // Shutdown flag
  bool mbShutDown;

  // Tracking state
  int mTrackingState;
  std::vector<MapPoint *> mTrackedMapPoints;
  std::vector<cv::KeyPoint> mTrackedKeyPointsUn;
  std::mutex mMutexState;

  std::shared_ptr<Settings> settings_;

  FpsEstimator fps_estimator_;
};

}  // namespace ORB_SLAM3
