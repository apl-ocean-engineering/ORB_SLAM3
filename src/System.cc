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

#include "System.h"

#include <pangolin/pangolin.h>

#include <algorithm>
#include <boost/archive/binary_iarchive.hpp>
#include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/string.hpp>
#include <cstdio>
#include <exception>
#include <iomanip>
#include <iostream>
#include <list>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "Converter.h"

namespace ORB_SLAM3 {

Verbose::eLevel Verbose::th = Verbose::VERBOSITY_NORMAL;

SystemFactory::Expected SystemFactory::create(
    const std::shared_ptr<Settings> &settings, bool initFr,
    const string &strSequence) {
  if (!settings->validate()) {
    return tl::make_unexpected(ExpectedError::fmt("Settings do not validate"));
  }

  // Cannot use make_shared with friend constructors?
  auto sys = std::shared_ptr<System>(new System(settings, initFr, strSequence));

  // Initialization must occur separately because we use shared_from_this
  if (!sys->initialize()) {
    return tl::make_unexpected(
        ExpectedError::fmt("Unable to initialize SLAM system"));
  }

  return sys;
}

SystemFactory::Expected SystemFactory::create(const std::string &configFile,
                                              const SensorType sensor,
                                              bool initFr,
                                              const string &strSequence) {
  auto exSettings = SettingsLoader::load(configFile, sensor);

  if (!exSettings) {
    return tl::make_unexpected(ExpectedError::fmt("Unable to load settings"));
  }

  return SystemFactory::create(exSettings.value(), initFr, strSequence);
}

SystemFactory::Expected SystemFactory::create(const std::string &configFile,
                                              const std::string &vocabFile,
                                              const SensorType sensor,
                                              bool initFr,
                                              const string &strSequence) {
  auto exSettings = SettingsLoader::load(configFile, sensor, vocabFile);

  if (!exSettings) {
    return tl::make_unexpected(ExpectedError::fmt("Unable to load settings"));
  }

  auto settings = exSettings.value();
  return SystemFactory::create(settings, initFr, strSequence);
}

//===================================================================

System::System(const std::shared_ptr<Settings> &settings, bool initFr,
               const string &strSequence)
    : enable_shared_from_this<System>(),
      mpViewer(),
      mbReset(false),
      mbResetActiveMap(false),
      mbActivateLocalizationMode(false),
      mbDeactivateLocalizationMode(false),
      mbShutDown(false),
      settings_(settings) {
  printBanner();
}

void System::printBanner() {
  // Output welcome message
  cout << endl
       << "ORB-SLAM3 Copyright (C) 2017-2020 Carlos Campos, Richard Elvira, "
          "Juan J. Gómez, José M.M. Montiel and Juan D. Tardós, University of "
          "Zaragoza."
       << endl
       << "ORB-SLAM2 Copyright (C) 2014-2016 Raúl Mur-Artal, José M.M. Montiel "
          "and Juan D. Tardós, University of Zaragoza."
       << endl
       << "This program comes with ABSOLUTELY NO WARRANTY;" << endl
       << "This is free software, and you are welcome to redistribute it"
       << endl
       << "under certain conditions. See LICENSE.txt." << endl
       << endl;

  oslog::info("Input sensor is type {}", sensorType().toString());
}

bool System::initialize(bool initFr, const string &strSequence) {
  const string mStrLoadAtlasFromFile = settings_->atlasLoadFile();

  cout << (*settings_) << endl;

  const bool activeLC = settings_->loopClosing_;
  const string vocabularyFilePath = settings_->strVocFile_;

  // Load ORB Vocabulary
  oslog::info("Loading ORB Vocabulary. This could take a while...");

  mpVocabulary = std::make_shared<ORBVocabulary>();
  bool bVocLoad = mpVocabulary->loadFromTextFile(vocabularyFilePath);
  if (!bVocLoad) {
    cerr << "Wrong path to vocabulary. " << endl;
    cerr << "Falied to open at: " << vocabularyFilePath << endl;
    return false;
  }
  oslog::info("Vocabulary loaded!");

  // Create KeyFrame Database
  mpKeyFrameDatabase = std::make_shared<KeyFrameDatabase>(mpVocabulary);

  if (mStrLoadAtlasFromFile.empty()) {
    // Create the Atlas
    oslog::info("Initializing Atlas from scratch ");
    mpAtlas = std::make_shared<Atlas>(0);
  } else {
    // Load the file with an earlier session
    // clock_t start = clock();
    oslog::info("Initializing Atlas from file: {}", mStrLoadAtlasFromFile);
    bool isRead = LoadAtlas(FileType::BINARY_FILE);

    if (!isRead) {
      oslog::error(
          "Unable to load Atlas file, please try with other session file or "
          "vocabulary file");
      return false;
    }

    mpAtlas->CreateNewMap();
  }

  if (sensorType().isImu()) mpAtlas->SetInertialSensor();

  // Only draw right image in stereo modes
  const bool frame_drawer_both = sensorType().isStereo();

  // Create Drawers. These are used by the Viewer
  mpFrameDrawer = std::make_shared<FrameDrawer>(mpAtlas, frame_drawer_both);
  mpMapDrawer = std::make_shared<MapDrawer>(mpAtlas, settings_);

  // Initialize the Tracking thread
  // (it will live in the main thread of execution, the one that called this
  // constructor)
  oslog::info("Seq. Name: {}", strSequence);
  mpTracker = std::make_shared<Tracking>(
      shared_from_this(), mpVocabulary, mpFrameDrawer, mpMapDrawer, mpAtlas,
      mpKeyFrameDatabase, settings_, strSequence);

  // Initialize the Local Mapping thread and launch
  mpLocalMapper = std::make_shared<LocalMapping>(
      shared_from_this(), mpAtlas, sensorType().isMonocular(),
      sensorType().isImu(), strSequence);
  mpLocalMapper->mInitFr = initFr;
  mpLocalMapper->mThFarPoints = settings_->thFarPoints();

  if (mpLocalMapper->mThFarPoints != 0) {
    oslog::info(
        "LocalMapping will discard points further than {} m from current "
        "camera",
        mpLocalMapper->mThFarPoints);
    mpLocalMapper->mbFarPoints = true;
  } else {
    oslog::info("LocalMapping will _not_ discard far points");
    mpLocalMapper->mbFarPoints = false;
  }

  mptLocalMapping =
      std::make_unique<thread>(&ORB_SLAM3::LocalMapping::Run, mpLocalMapper);

  // Initialize the Loop Closing thread and launch
  mpLoopCloser = std::make_shared<LoopClosing>(
      mpAtlas, mpKeyFrameDatabase, mpVocabulary,
      sensorType() != SensorType::MONOCULAR, activeLC);
  mptLoopClosing =
      std::make_unique<thread>(&ORB_SLAM3::LoopClosing::Run, mpLoopCloser);

  // Set pointers between threads
  mpTracker->SetLocalMapper(mpLocalMapper);
  mpTracker->SetLoopClosing(mpLoopCloser);

  mpLocalMapper->SetTracker(mpTracker);
  mpLocalMapper->SetLoopCloser(mpLoopCloser);

  mpLoopCloser->SetTracker(mpTracker);
  mpLoopCloser->SetLocalMapper(mpLocalMapper);

  // Initialize the Viewer thread and launch
  oslog::info("Viewer enabled: {}", settings_->useViewer_ ? "YES" : "NO");
  if (settings_->useViewer_) {
    mpViewer = std::make_shared<Viewer>(this, mpFrameDrawer, mpMapDrawer,
                                        mpTracker, settings_);
    mptViewer = std::make_unique<thread>(&Viewer::Run, mpViewer);
    mpTracker->SetViewer(mpViewer);
    mpLoopCloser->mpViewer = mpViewer;
    mpViewer->both = mpFrameDrawer->both;
  }

  // Fix verbosity
  Verbose::SetTh(Verbose::VERBOSITY_DEBUG);

  return true;
}

Sophus::SE3f System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight,
                                 const double &timestamp,
                                 const vector<IMU::Point> &vImuMeas,
                                 string filename) {
  if (!sensorType().isStereo()) {
    oslog::error(
        "ERROR: you called TrackStereo but input sensor was not set to "
        "Stereo nor Stereo-Inertial.");
    exit(-1);
  }

  cv::Mat imLeftToFeed, imRightToFeed;
  if (settings_ && settings_->needToRectify()) {
    cv::Mat M1l = settings_->M1l();
    cv::Mat M2l = settings_->M2l();
    cv::Mat M1r = settings_->M1r();
    cv::Mat M2r = settings_->M2r();

    cv::remap(imLeft, imLeftToFeed, M1l, M2l, cv::INTER_LINEAR);
    cv::remap(imRight, imRightToFeed, M1r, M2r, cv::INTER_LINEAR);
  } else if (settings_ && settings_->needToResize()) {
    cv::resize(imLeft, imLeftToFeed, settings_->newImSize());
    cv::resize(imRight, imRightToFeed, settings_->newImSize());
  } else {
    imLeftToFeed = imLeft.clone();
    imRightToFeed = imRight.clone();
  }

  processLocalizationModeChange();
  processReset();

  fps_estimator_.pushTimestamp(timestamp);
  oslog::debug("[System] Current FPS estimate {:2f}", fps_estimator_.fps());

  if (sensorType().isImu()) {
    for (auto const &imuMeas : vImuMeas) {
      mpTracker->GrabImuData(imuMeas);
    }
  }

  Sophus::SE3f Tcw = mpTracker->GrabImageStereo(imLeftToFeed, imRightToFeed,
                                                timestamp, filename);

  updateTrackingState();

  return Tcw;
}

Sophus::SE3f System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap,
                               const double &timestamp,
                               const vector<IMU::Point> &vImuMeas,
                               string filename) {
  if (!sensorType().isRGBD()) {
    cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD."
         << endl;
    exit(-1);
  }

  cv::Mat imToFeed = im.clone();
  cv::Mat imDepthToFeed = depthmap.clone();
  if (settings_ && settings_->needToResize()) {
    cv::Mat resizedIm;
    cv::resize(im, resizedIm, settings_->newImSize());
    imToFeed = resizedIm;

    cv::resize(depthmap, imDepthToFeed, settings_->newImSize());
  }

  processLocalizationModeChange();
  processReset();

  fps_estimator_.pushTimestamp(timestamp);

  if (sensorType().isImu()) {
    for (auto const &imuMeas : vImuMeas) {
      mpTracker->GrabImuData(imuMeas);
    }
  }

  Sophus::SE3f Tcw =
      mpTracker->GrabImageRGBD(imToFeed, imDepthToFeed, timestamp, filename);

  updateTrackingState();

  return Tcw;
}

Sophus::SE3f System::TrackMonocular(const cv::Mat &im, const double &timestamp,
                                    const vector<IMU::Point> &vImuMeas,
                                    string filename) {
  {
    unique_lock<mutex> lock(mMutexReset);
    if (mbShutDown) return Sophus::SE3f();
  }

  if (!sensorType().isMonocular()) {
    cerr << "ERROR: you called TrackMonocular but input sensor was not set to "
            "Monocular nor Monocular-Inertial."
         << endl;
    exit(-1);
  }

  cv::Mat imToFeed = im.clone();
  if (settings_ && settings_->needToResize()) {
    cv::Mat resizedIm;
    cv::resize(im, resizedIm, settings_->newImSize());
    imToFeed = resizedIm;
  }

  processLocalizationModeChange();
  processReset();

  fps_estimator_.pushTimestamp(timestamp);

  if (sensorType().isImu()) {
    for (auto const &imuMeas : vImuMeas) {
      mpTracker->GrabImuData(imuMeas);
    }
  }

  Sophus::SE3f Tcw =
      mpTracker->GrabImageMonocular(imToFeed, timestamp, filename);

  updateTrackingState();

  return Tcw;
}

//===============================================

void System::ActivateLocalizationMode() {
  unique_lock<mutex> lock(mMutexMode);
  mbActivateLocalizationMode = true;
}

void System::DeactivateLocalizationMode() {
  unique_lock<mutex> lock(mMutexMode);
  mbDeactivateLocalizationMode = true;
}

void System::processLocalizationModeChange() {
  unique_lock<mutex> lock(mMutexMode);
  if (mbActivateLocalizationMode) {
    mpLocalMapper->RequestStop();

    // Wait until Local Mapping has effectively stopped
    while (!mpLocalMapper->isStopped()) {
      usleep(1000);
    }

    mpTracker->InformOnlyTracking(true);
    mbActivateLocalizationMode = false;
  }
  if (mbDeactivateLocalizationMode) {
    mpTracker->InformOnlyTracking(false);
    mpLocalMapper->Release();
    mbDeactivateLocalizationMode = false;
  }
}

//===============================================

void System::Reset() {
  unique_lock<mutex> lock(mMutexReset);
  mbReset = true;
}

void System::ResetActiveMap() {
  unique_lock<mutex> lock(mMutexReset);
  mbResetActiveMap = true;
}

void System::processReset() {
  unique_lock<mutex> lock(mMutexReset);
  if (mbReset) {
    mpTracker->Reset();
    mbReset = false;
    mbResetActiveMap = false;
  } else if (mbResetActiveMap) {
    mpTracker->ResetActiveMap();
    mbResetActiveMap = false;
  }
}

//===============================================

void System::updateTrackingState() {
  unique_lock<mutex> lock2(mMutexState);
  mTrackingState = mpTracker->mState;
  mTrackedMapPoints = mpTracker->mCurrentFrame->mvpMapPoints;
  mTrackedKeyPointsUn = mpTracker->mCurrentFrame->mvKeysUn;
}

//===============================================

bool System::MapChanged() {
  static int n = 0;
  int curn = mpAtlas->GetLastBigChangeIdx();
  if (n < curn) {
    n = curn;
    return true;
  } else {
    return false;
  }
}

void System::Shutdown() {
  {
    unique_lock<mutex> lock(mMutexReset);
    mbShutDown = true;
  }

  oslog::warn("Shutdown");

  mpLocalMapper->RequestFinish();
  mpLoopCloser->RequestFinish();
  /*if(mpViewer)
  {
      mpViewer->RequestFinish();
      while(!mpViewer->isFinished())
          usleep(5000);
  }*/

  // Wait until all thread have effectively stopped
  while (!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() ||
         mpLoopCloser->isRunningGBA()) {
    if (!mpLocalMapper->isFinished()) {
      oslog::warn("mpLocalMapper is not finished");
    }

    if (!mpLoopCloser->isFinished()) {
      oslog::warn("mpLoopCloser is not finished");
    }

    if (mpLoopCloser->isRunningGBA()) {
      oslog::warn("mpLoopCloser is running GBA");
    }

    oslog::warn(" .... waiting");
    usleep(5000);
  }

  oslog::warn("All threads finished");

  const string mStrSaveAtlasToFile = settings_->atlasSaveFile();
  if (!mStrSaveAtlasToFile.empty()) {
    Verbose::PrintMess("Atlas saving to file " + mStrSaveAtlasToFile,
                       Verbose::VERBOSITY_NORMAL);
    SaveAtlas(FileType::BINARY_FILE);
  }

  if (mpViewer) pangolin::BindToContext("ORB-SLAM3: Map Viewer");

#ifdef REGISTER_TIMES
  mpTracker->PrintTimeStats();
#endif
}

bool System::isShutDown() {
  unique_lock<mutex> lock(mMutexReset);
  return mbShutDown;
}

int System::GetTrackingState() {
  unique_lock<mutex> lock(mMutexState);
  return mTrackingState;
}

vector<MapPoint *> System::GetTrackedMapPoints() {
  unique_lock<mutex> lock(mMutexState);
  return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::GetTrackedKeyPointsUn() {
  unique_lock<mutex> lock(mMutexState);
  return mTrackedKeyPointsUn;
}

double System::GetTimeFromIMUInit() {
  double aux = mpLocalMapper->GetCurrKFTime() - mpLocalMapper->mFirstTs;
  if ((aux > 0.) && mpAtlas->isImuInitialized())
    return mpLocalMapper->GetCurrKFTime() - mpLocalMapper->mFirstTs;
  else
    return 0.f;
}

bool System::isLost() {
  if (!mpAtlas->isImuInitialized()) {
    return false;
  } else {
    if ((mpTracker->mState ==
         Tracking::LOST))  // ||(mpTracker->mState==Tracking::RECENTLY_LOST))
      return true;
    else
      return false;
  }
}

bool System::isFinished() { return (GetTimeFromIMUInit() > 0.1); }

void System::ChangeDataset() {
  if (mpAtlas->GetCurrentMap()->KeyFramesInMap() < 12) {
    mpTracker->ResetActiveMap();
  } else {
    mpTracker->CreateMapInAtlas();
  }

  mpTracker->NewDataset();
}

float System::GetImageScale() { return mpTracker->GetImageScale(); }

#ifdef REGISTER_TIMES
void System::InsertRectTime(double &time) {
  mpTracker->vdRectStereo_ms.push_back(time);
}

void System::InsertResizeTime(double &time) {
  mpTracker->vdResizeImage_ms.push_back(time);
}

void System::InsertTrackTime(double &time) {
  mpTracker->vdTrackTotal_ms.push_back(time);
}
#endif

}  // namespace ORB_SLAM3
