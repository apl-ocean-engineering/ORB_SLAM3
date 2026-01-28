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

#include <openssl/md5.h>
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

  oslog::info("Input sensor was set to: {}", sensorType().toString());
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

  cout << "Shutdown" << endl;

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

void System::SaveTrajectoryTUM(const string &filename) {
  cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
  if (sensorType() == SensorType::MONOCULAR) {
    cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
    return;
  }

  vector<std::shared_ptr<KeyFrame>> vpKFs = mpAtlas->GetAllKeyFrames();
  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  Sophus::SE3f Two = vpKFs[0]->GetPoseInverse();

  ofstream f;
  f.open(filename.c_str());
  f << fixed;

  // Frame pose is stored relative to its reference keyframe (which is optimized
  // by BA and pose graph). We need to get first the keyframe pose and then
  // concatenate the relative transformation. Frames not localized (tracking
  // failure) are not saved.

  // For each frame we have a reference keyframe (lRit), the timestamp (lT) and
  // a flag which is true when tracking failed (lbL).
  list<std::shared_ptr<KeyFrame>>::iterator lRit =
      mpTracker->mlpReferences.begin();
  list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
  list<bool>::iterator lbL = mpTracker->mlbLost.begin();
  for (list<Sophus::SE3f>::iterator
           lit = mpTracker->mlRelativeFramePoses.begin(),
           lend = mpTracker->mlRelativeFramePoses.end();
       lit != lend; lit++, lRit++, lT++, lbL++) {
    if (*lbL) continue;

    std::shared_ptr<KeyFrame> pKF = *lRit;

    Sophus::SE3f Trw;

    // If the reference keyframe was culled, traverse the spanning tree to get a
    // suitable keyframe.
    while (pKF->isBad()) {
      Trw = Trw * pKF->mTcp;
      pKF = pKF->GetParent();
    }

    Trw = Trw * pKF->GetPose() * Two;

    Sophus::SE3f Tcw = (*lit) * Trw;
    Sophus::SE3f Twc = Tcw.inverse();

    Eigen::Vector3f twc = Twc.translation();
    Eigen::Quaternionf q = Twc.unit_quaternion();

    f << setprecision(6) << *lT << " " << setprecision(9) << twc(0) << " "
      << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " " << q.z()
      << " " << q.w() << endl;
  }
  f.close();
  // cout << endl << "trajectory saved!" << endl;
}

void System::SaveKeyFrameTrajectoryTUM(const string &filename) {
  cout << endl
       << "Saving keyframe trajectory to " << filename << " ..." << endl;

  vector<std::shared_ptr<KeyFrame>> vpKFs = mpAtlas->GetAllKeyFrames();
  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  ofstream f;
  f.open(filename.c_str());
  f << fixed;

  for (auto pKF : vpKFs) {
    if (pKF->isBad()) continue;

    Sophus::SE3f Twc = pKF->GetPoseInverse();
    Eigen::Quaternionf q = Twc.unit_quaternion();
    Eigen::Vector3f t = Twc.translation();
    f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t(0)
      << " " << t(1) << " " << t(2) << " " << q.x() << " " << q.y() << " "
      << q.z() << " " << q.w() << endl;
  }

  f.close();
}

void System::SaveTrajectoryEuRoC(const string &filename) {
  cout << endl << "Saving trajectory to " << filename << " ..." << endl;
  /*if(sensorType()==MONOCULAR)
  {
      cerr << "ERROR: SaveTrajectoryEuRoC cannot be used for monocular." <<
  endl; return;
  }*/

  vector<std::shared_ptr<Map>> vpMaps = mpAtlas->GetAllMaps();
  size_t numMaxKFs = 0;
  std::shared_ptr<Map> pBiggerMap;
  std::cout << "There are " << std::to_string(vpMaps.size())
            << " maps in the atlas" << std::endl;
  for (auto pMap : vpMaps) {
    std::cout << "  Map " << std::to_string(pMap->GetId()) << " has "
              << std::to_string(pMap->GetAllKeyFrames().size()) << " KFs"
              << std::endl;
    if (pMap->GetAllKeyFrames().size() > numMaxKFs) {
      numMaxKFs = pMap->GetAllKeyFrames().size();
      pBiggerMap = pMap;
    }
  }

  auto vpKFs = pBiggerMap->GetAllKeyFrames();
  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  Sophus::SE3f
      Twb;  // Can be word to cam0 or world to b depending on IMU or not.
  if (sensorType().isImu()) {
    Twb = vpKFs[0]->GetImuPose();
  } else {
    Twb = vpKFs[0]->GetPoseInverse();
  }

  ofstream f;
  f.open(filename.c_str());
  // cout << "file open" << endl;
  f << fixed;

  // Frame pose is stored relative to its reference keyframe (which is optimized
  // by BA and pose graph). We need to get first the keyframe pose and then
  // concatenate the relative transformation. Frames not localized (tracking
  // failure) are not saved.

  // For each frame we have a reference keyframe (lRit), the timestamp (lT) and
  // a flag which is true when tracking failed (lbL).
  auto lRit = mpTracker->mlpReferences.begin();
  list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
  list<bool>::iterator lbL = mpTracker->mlbLost.begin();

  // cout << "size mlpReferences: " << mpTracker->mlpReferences.size() << endl;
  // cout << "size mlRelativeFramePoses: " <<
  // mpTracker->mlRelativeFramePoses.size() << endl; cout << "size
  // mpTracker->mlFrameTimes: " << mpTracker->mlFrameTimes.size() << endl; cout
  // << "size mpTracker->mlbLost: " << mpTracker->mlbLost.size() << endl;

  for (auto lit = mpTracker->mlRelativeFramePoses.begin(),
            lend = mpTracker->mlRelativeFramePoses.end();
       lit != lend; lit++, lRit++, lT++, lbL++) {
    // cout << "1" << endl;
    if (*lbL) continue;

    std::shared_ptr<KeyFrame> pKF = *lRit;
    // cout << "KF: " << pKF->mnId << endl;

    Sophus::SE3f Trw;

    // If the reference keyframe was culled, traverse the spanning tree to get a
    // suitable keyframe.
    if (!pKF) continue;

    // cout << "2.5" << endl;

    while (pKF->isBad()) {
      // cout << " 2.bad" << endl;
      Trw = Trw * pKF->mTcp;
      pKF = pKF->GetParent();
      // cout << "--Parent KF: " << pKF->mnId << endl;
    }

    if (!pKF || pKF->GetMap() != pBiggerMap) {
      // cout << "--Parent KF is from another map" << endl;
      continue;
    }

    // cout << "3" << endl;

    Trw = Trw * pKF->GetPose() *
          Twb;  // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

    // cout << "4" << endl;

    if (sensorType().isImu()) {
      Sophus::SE3f Twb = (pKF->mImuCalib.mTbc * (*lit) * Trw).inverse();
      Eigen::Quaternionf q = Twb.unit_quaternion();
      Eigen::Vector3f twb = Twb.translation();
      f << setprecision(6) << 1e9 * (*lT) << " " << setprecision(9) << twb(0)
        << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " "
        << q.z() << " " << q.w() << endl;
    } else {
      Sophus::SE3f Twc = ((*lit) * Trw).inverse();
      Eigen::Quaternionf q = Twc.unit_quaternion();
      Eigen::Vector3f twc = Twc.translation();
      f << setprecision(6) << 1e9 * (*lT) << " " << setprecision(9) << twc(0)
        << " " << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " "
        << q.z() << " " << q.w() << endl;
    }

    // cout << "5" << endl;
  }
  // cout << "end saving trajectory" << endl;
  f.close();
  cout << endl << "End of saving trajectory to " << filename << " ..." << endl;
}

void System::SaveTrajectoryEuRoC(const string &filename,
                                 const std::shared_ptr<Map> &pMap) {
  cout << endl
       << "Saving trajectory of map " << pMap->GetId() << " to " << filename
       << " ..." << endl;
  /*if(sensorType()==MONOCULAR)
  {
      cerr << "ERROR: SaveTrajectoryEuRoC cannot be used for monocular." <<
  endl; return;
  }*/

  auto vpKFs = pMap->GetAllKeyFrames();
  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  Sophus::SE3f
      Twb;  // Can be word to cam0 or world to b dependingo on IMU or not.
  if (sensorType().isImu()) {
    Twb = vpKFs[0]->GetImuPose();
  } else {
    Twb = vpKFs[0]->GetPoseInverse();
  }
  ofstream f;
  f.open(filename.c_str());
  // cout << "file open" << endl;
  f << fixed;

  // Frame pose is stored relative to its reference keyframe (which is optimized
  // by BA and pose graph). We need to get first the keyframe pose and then
  // concatenate the relative transformation. Frames not localized (tracking
  // failure) are not saved.

  // For each frame we have a reference keyframe (lRit), the timestamp (lT) and
  // a flag which is true when tracking failed (lbL).
  auto lRit = mpTracker->mlpReferences.begin();
  list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
  list<bool>::iterator lbL = mpTracker->mlbLost.begin();

  // cout << "size mlpReferences: " << mpTracker->mlpReferences.size() << endl;
  // cout << "size mlRelativeFramePoses: " <<
  // mpTracker->mlRelativeFramePoses.size() << endl; cout << "size
  // mpTracker->mlFrameTimes: " << mpTracker->mlFrameTimes.size() << endl; cout
  // << "size mpTracker->mlbLost: " << mpTracker->mlbLost.size() << endl;

  for (auto lit = mpTracker->mlRelativeFramePoses.begin(),
            lend = mpTracker->mlRelativeFramePoses.end();
       lit != lend; lit++, lRit++, lT++, lbL++) {
    // cout << "1" << endl;
    if (*lbL) continue;

    std::shared_ptr<KeyFrame> pKF = *lRit;
    // cout << "KF: " << pKF->mnId << endl;

    Sophus::SE3f Trw;

    // If the reference keyframe was culled, traverse the spanning tree to get a
    // suitable keyframe.
    if (!pKF) continue;

    // cout << "2.5" << endl;

    while (pKF->isBad()) {
      // cout << " 2.bad" << endl;
      Trw = Trw * pKF->mTcp;
      pKF = pKF->GetParent();
      // cout << "--Parent KF: " << pKF->mnId << endl;
    }

    if (!pKF || pKF->GetMap() != pMap) {
      // cout << "--Parent KF is from another map" << endl;
      continue;
    }

    // cout << "3" << endl;

    Trw = Trw * pKF->GetPose() *
          Twb;  // Tcp*Tpw*Twb0=Tcb0 where b0 is the new world reference

    // cout << "4" << endl;

    if (sensorType().isImu()) {
      Sophus::SE3f Twb = (pKF->mImuCalib.mTbc * (*lit) * Trw).inverse();
      Eigen::Quaternionf q = Twb.unit_quaternion();
      Eigen::Vector3f twb = Twb.translation();
      f << setprecision(6) << 1e9 * (*lT) << " " << setprecision(9) << twb(0)
        << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " "
        << q.z() << " " << q.w() << endl;
    } else {
      Sophus::SE3f Twc = ((*lit) * Trw).inverse();
      Eigen::Quaternionf q = Twc.unit_quaternion();
      Eigen::Vector3f twc = Twc.translation();
      f << setprecision(6) << 1e9 * (*lT) << " " << setprecision(9) << twc(0)
        << " " << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " "
        << q.z() << " " << q.w() << endl;
    }

    // cout << "5" << endl;
  }
  // cout << "end saving trajectory" << endl;
  f.close();
  cout << endl << "End of saving trajectory to " << filename << " ..." << endl;
}

/*void System::SaveTrajectoryEuRoC(const string &filename)
{

    cout << endl << "Saving trajectory to " << filename << " ..." << endl;
    if(sensorType()==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryEuRoC cannot be used for monocular." <<
endl; return;
    }

    vector<Map*> vpMaps = mpAtlas->GetAllMaps();
    Map* pBiggerMap;
    int numMaxKFs = 0;
    for(Map* pMap :vpMaps)
    {
        if(pMap->GetAllKeyFrames().size() > numMaxKFs)
        {
            numMaxKFs = pMap->GetAllKeyFrames().size();
            pBiggerMap = pMap;
        }
    }

    vector<KeyFrame*> vpKFs = pBiggerMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    Sophus::SE3f Twb; // Can be word to cam0 or world to b dependingo on IMU or
not. if (sensorType()==IMU_MONOCULAR || sensorType()==IMU_STEREO ||
sensorType()==IMU_RGBD) Twb = vpKFs[0]->GetImuPose_(); else Twb =
vpKFs[0]->GetPoseInverse_();

    ofstream f;
    f.open(filename.c_str());
    // cout << "file open" << endl;
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is
optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative
transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT)
and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM3::KeyFrame*>::iterator lRit =
mpTracker->mlpReferences.begin(); list<double>::iterator lT =
mpTracker->mlFrameTimes.begin(); list<bool>::iterator lbL =
mpTracker->mlbLost.begin();

    //cout << "size mlpReferences: " << mpTracker->mlpReferences.size() << endl;
    //cout << "size mlRelativeFramePoses: " <<
mpTracker->mlRelativeFramePoses.size() << endl;
    //cout << "size mpTracker->mlFrameTimes: " << mpTracker->mlFrameTimes.size()
<< endl;
    //cout << "size mpTracker->mlbLost: " << mpTracker->mlbLost.size() << endl;


    for(list<Sophus::SE3f>::iterator
lit=mpTracker->mlRelativeFramePoses.begin(),
        lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++,
lT++, lbL++)
    {
        //cout << "1" << endl;
        if(*lbL)
            continue;


        KeyFrame* pKF = *lRit;
        //cout << "KF: " << pKF->mnId << endl;

        Sophus::SE3f Trw;

        // If the reference keyframe was culled, traverse the spanning tree to
get a suitable keyframe. if (!pKF) continue;

        //cout << "2.5" << endl;

        while(pKF->isBad())
        {
            //cout << " 2.bad" << endl;
            Trw = Trw * pKF->mTcp;
            pKF = pKF->GetParent();
            //cout << "--Parent KF: " << pKF->mnId << endl;
        }

        if(!pKF || pKF->GetMap() != pBiggerMap)
        {
            //cout << "--Parent KF is from another map" << endl;
            continue;
        }

        //cout << "3" << endl;

        Trw = Trw * pKF->GetPose()*Twb; // Tcp*Tpw*Twb0=Tcb0 where b0 is the new
world reference

        // cout << "4" << endl;


        if (sensorType() == IMU_MONOCULAR || sensorType() == IMU_STEREO ||
sensorType()==IMU_RGBD)
        {
            Sophus::SE3f Tbw = pKF->mImuCalib.Tbc_ * (*lit) * Trw;
            Sophus::SE3f Twb = Tbw.inverse();

            Eigen::Vector3f twb = Twb.translation();
            Eigen::Quaternionf q = Twb.unit_quaternion();
            f << setprecision(6) << 1e9*(*lT) << " " <<  setprecision(9) <<
twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " " << q.y() << " "
<< q.z() << " " << q.w() << endl;
        }
        else
        {
            Sophus::SE3f Tcw = (*lit) * Trw;
            Sophus::SE3f Twc = Tcw.inverse();

            Eigen::Vector3f twc = Twc.translation();
            Eigen::Quaternionf q = Twc.unit_quaternion();
            f << setprecision(6) << 1e9*(*lT) << " " <<  setprecision(9) <<
twc(0) << " " << twc(1) << " " << twc(2) << " " << q.x() << " " << q.y() << " "
<< q.z() << " " << q.w() << endl;
        }

        // cout << "5" << endl;
    }
    //cout << "end saving trajectory" << endl;
    f.close();
    cout << endl << "End of saving trajectory to " << filename << " ..." <<
endl;
}*/

/*void System::SaveKeyFrameTrajectoryEuRoC_old(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." <<
endl;

    vector<Map*> vpMaps = mpAtlas->GetAllMaps();
    Map* pBiggerMap;
    int numMaxKFs = 0;
    for(Map* pMap :vpMaps)
    {
        if(pMap->GetAllKeyFrames().size() > numMaxKFs)
        {
            numMaxKFs = pMap->GetAllKeyFrames().size();
            pBiggerMap = pMap;
        }
    }

    vector<KeyFrame*> vpKFs = pBiggerMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;
        if (sensorType() == IMU_MONOCULAR || sensorType() == IMU_STEREO ||
sensorType()==IMU_RGBD)
        {
            cv::Mat R = pKF->GetImuRotation().t();
            vector<float> q = Converter::toQuaternion(R);
            cv::Mat twb = pKF->GetImuPosition();
            f << setprecision(6) << 1e9*pKF->mTimeStamp  << " " <<
setprecision(9) << twb.at<float>(0) << " " << twb.at<float>(1) << " " <<
twb.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] <<
endl;

        }
        else
        {
            cv::Mat R = pKF->GetRotation();
            vector<float> q = Converter::toQuaternion(R);
            cv::Mat t = pKF->GetCameraCenter();
            f << setprecision(6) << 1e9*pKF->mTimeStamp << " " <<
setprecision(9) << t.at<float>(0) << " " << t.at<float>(1) << " " <<
t.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] <<
endl;
        }
    }
    f.close();
}*/

void System::SaveKeyFrameTrajectoryEuRoC(const string &filename) {
  cout << endl
       << "Saving keyframe trajectory to " << filename << " ..." << endl;

  vector<std::shared_ptr<Map>> vpMaps = mpAtlas->GetAllMaps();
  std::shared_ptr<Map> pBiggerMap;
  size_t numMaxKFs = 0;
  for (auto pMap : vpMaps) {
    if (pMap && pMap->GetAllKeyFrames().size() > numMaxKFs) {
      numMaxKFs = pMap->GetAllKeyFrames().size();
      pBiggerMap = pMap;
    }
  }

  if (!pBiggerMap) {
    std::cout << "There is not a map!!" << std::endl;
    return;
  }

  auto vpKFs = pBiggerMap->GetAllKeyFrames();
  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  ofstream f;
  f.open(filename.c_str());
  f << fixed;

  for (auto pKF : vpKFs) {
    // pKF->SetPose(pKF->GetPose()*Two);

    if (!pKF || pKF->isBad()) continue;
    if (sensorType().isImu()) {
      Sophus::SE3f Twb = pKF->GetImuPose();
      Eigen::Quaternionf q = Twb.unit_quaternion();
      Eigen::Vector3f twb = Twb.translation();
      f << setprecision(6) << 1e9 * pKF->mTimeStamp << " " << setprecision(9)
        << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " "
        << q.y() << " " << q.z() << " " << q.w() << endl;

    } else {
      Sophus::SE3f Twc = pKF->GetPoseInverse();
      Eigen::Quaternionf q = Twc.unit_quaternion();
      Eigen::Vector3f t = Twc.translation();
      f << setprecision(6) << 1e9 * pKF->mTimeStamp << " " << setprecision(9)
        << t(0) << " " << t(1) << " " << t(2) << " " << q.x() << " " << q.y()
        << " " << q.z() << " " << q.w() << endl;
    }
  }
  f.close();
}

void System::SaveKeyFrameTrajectoryEuRoC(const string &filename,
                                         const std::shared_ptr<Map> &pMap) {
  cout << endl
       << "Saving keyframe trajectory of map " << pMap->GetId() << " to "
       << filename << " ..." << endl;

  auto vpKFs = pMap->GetAllKeyFrames();
  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  ofstream f;
  f.open(filename.c_str());
  f << fixed;

  for (auto pKF : vpKFs) {
    if (!pKF || pKF->isBad()) continue;

    if (sensorType().isImu()) {
      Sophus::SE3f Twb = pKF->GetImuPose();
      Eigen::Quaternionf q = Twb.unit_quaternion();
      Eigen::Vector3f twb = Twb.translation();
      f << setprecision(6) << 1e9 * pKF->mTimeStamp << " " << setprecision(9)
        << twb(0) << " " << twb(1) << " " << twb(2) << " " << q.x() << " "
        << q.y() << " " << q.z() << " " << q.w() << endl;
    } else {
      Sophus::SE3f Twc = pKF->GetPoseInverse();
      Eigen::Quaternionf q = Twc.unit_quaternion();
      Eigen::Vector3f t = Twc.translation();
      f << setprecision(6) << 1e9 * pKF->mTimeStamp << " " << setprecision(9)
        << t(0) << " " << t(1) << " " << t(2) << " " << q.x() << " " << q.y()
        << " " << q.z() << " " << q.w() << endl;
    }
  }

  f.close();
}

/*void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." <<
endl; if(sensorType()==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." <<
endl; return;
    }

    vector<KeyFrame*> vpKFs = mpAtlas->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is
optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative
transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT)
and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM3::KeyFrame*>::iterator lRit =
mpTracker->mlpReferences.begin(); list<double>::iterator lT =
mpTracker->mlFrameTimes.begin(); for(list<cv::Mat>::iterator
lit=mpTracker->mlRelativeFramePoses.begin(),
lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM3::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
            Trw = Trw * Converter::toCvMat(pKF->mTcp.matrix());
            pKF = pKF->GetParent();
        }

        Trw = Trw * pKF->GetPoseCv() * Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)
<< " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " <<
Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " << Rwc.at<float>(2,0) << "
" << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  <<
twc.at<float>(2) << endl;
    }
    f.close();
}*/

void System::SaveTrajectoryKITTI(const string &filename) {
  cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
  if (sensorType() == SensorType::MONOCULAR) {
    cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
    return;
  }

  auto vpKFs = mpAtlas->GetAllKeyFrames();
  sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

  // Transform all keyframes so that the first keyframe is at the origin.
  // After a loop closure the first keyframe might not be at the origin.
  Sophus::SE3f Tow = vpKFs[0]->GetPoseInverse();

  ofstream f;
  f.open(filename.c_str());
  f << fixed;

  // Frame pose is stored relative to its reference keyframe (which is optimized
  // by BA and pose graph). We need to get first the keyframe pose and then
  // concatenate the relative transformation. Frames not localized (tracking
  // failure) are not saved.

  // For each frame we have a reference keyframe (lRit), the timestamp (lT) and
  // a flag which is true when tracking failed (lbL).
  auto lRit = mpTracker->mlpReferences.begin();
  list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
  for (list<Sophus::SE3f>::iterator
           lit = mpTracker->mlRelativeFramePoses.begin(),
           lend = mpTracker->mlRelativeFramePoses.end();
       lit != lend; lit++, lRit++, lT++) {
    auto pKF = *lRit;

    Sophus::SE3f Trw;

    if (!pKF) continue;

    while (pKF->isBad()) {
      Trw = Trw * pKF->mTcp;
      pKF = pKF->GetParent();
    }

    Trw = Trw * pKF->GetPose() * Tow;

    Sophus::SE3f Tcw = (*lit) * Trw;
    Sophus::SE3f Twc = Tcw.inverse();
    Eigen::Matrix3f Rwc = Twc.rotationMatrix();
    Eigen::Vector3f twc = Twc.translation();

    f << setprecision(9) << Rwc(0, 0) << " " << Rwc(0, 1) << " " << Rwc(0, 2)
      << " " << twc(0) << " " << Rwc(1, 0) << " " << Rwc(1, 1) << " "
      << Rwc(1, 2) << " " << twc(1) << " " << Rwc(2, 0) << " " << Rwc(2, 1)
      << " " << Rwc(2, 2) << " " << twc(2) << endl;
  }
  f.close();
}

void System::SaveDebugData(const int &initIdx) {
  // 0. Save initialization trajectory
  SaveTrajectoryEuRoC("init_FrameTrajectoy_" +
                      to_string(mpLocalMapper->mInitSect) + "_" +
                      to_string(initIdx) + ".txt");

  // 1. Save scale
  ofstream f;
  f.open("init_Scale_" + to_string(mpLocalMapper->mInitSect) + ".txt",
         ios_base::app);
  f << fixed;
  f << mpLocalMapper->mScale << endl;
  f.close();

  // 2. Save gravity direction
  f.open("init_GDir_" + to_string(mpLocalMapper->mInitSect) + ".txt",
         ios_base::app);
  f << fixed;
  f << mpLocalMapper->mRwg(0, 0) << "," << mpLocalMapper->mRwg(0, 1) << ","
    << mpLocalMapper->mRwg(0, 2) << endl;
  f << mpLocalMapper->mRwg(1, 0) << "," << mpLocalMapper->mRwg(1, 1) << ","
    << mpLocalMapper->mRwg(1, 2) << endl;
  f << mpLocalMapper->mRwg(2, 0) << "," << mpLocalMapper->mRwg(2, 1) << ","
    << mpLocalMapper->mRwg(2, 2) << endl;
  f.close();

  // 3. Save computational cost
  f.open("init_CompCost_" + to_string(mpLocalMapper->mInitSect) + ".txt",
         ios_base::app);
  f << fixed;
  f << mpLocalMapper->mCostTime << endl;
  f.close();

  // 4. Save biases
  f.open("init_Biases_" + to_string(mpLocalMapper->mInitSect) + ".txt",
         ios_base::app);
  f << fixed;
  f << mpLocalMapper->mbg(0) << "," << mpLocalMapper->mbg(1) << ","
    << mpLocalMapper->mbg(2) << endl;
  f << mpLocalMapper->mba(0) << "," << mpLocalMapper->mba(1) << ","
    << mpLocalMapper->mba(2) << endl;
  f.close();

  // 5. Save covariance matrix
  f.open("init_CovMatrix_" + to_string(mpLocalMapper->mInitSect) + "_" +
             to_string(initIdx) + ".txt",
         ios_base::app);
  f << fixed;
  for (int i = 0; i < mpLocalMapper->mcovInertial.rows(); i++) {
    for (int j = 0; j < mpLocalMapper->mcovInertial.cols(); j++) {
      if (j != 0) f << ",";
      f << setprecision(15) << mpLocalMapper->mcovInertial(i, j);
    }
    f << endl;
  }
  f.close();

  // 6. Save initialization time
  f.open("init_Time_" + to_string(mpLocalMapper->mInitSect) + ".txt",
         ios_base::app);
  f << fixed;
  f << mpLocalMapper->mInitTime << endl;
  f.close();
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

void System::SaveAtlas(int type) {
  const string mStrSaveAtlasToFile = settings_->atlasSaveFile();

  if (!mStrSaveAtlasToFile.empty()) {
    // clock_t start = clock();

    // Save the current session
    mpAtlas->PreSave();

    string pathSaveFileName = "./";
    pathSaveFileName = pathSaveFileName.append(mStrSaveAtlasToFile);
    pathSaveFileName = pathSaveFileName.append(".osa");

    const string vocabularyFilePath = settings_->strVocFile_;

    string strVocabularyChecksum =
        CalculateCheckSum(vocabularyFilePath, TEXT_FILE);
    std::size_t found = vocabularyFilePath.find_last_of("/\\");
    string strVocabularyName = vocabularyFilePath.substr(found + 1);

    if (type == TEXT_FILE) {
      // File text

      cout << "Starting to write the save text file " << endl;
      std::remove(pathSaveFileName.c_str());
      std::ofstream ofs(pathSaveFileName, std::ios::binary);
      boost::archive::text_oarchive oa(ofs);

      oa << strVocabularyName;
      oa << strVocabularyChecksum;
      oa << mpAtlas;
      cout << "End to write the save text file" << endl;
    } else if (type == BINARY_FILE) {
      // File binary

      cout << "Starting to write the save binary file" << endl;
      std::remove(pathSaveFileName.c_str());
      std::ofstream ofs(pathSaveFileName, std::ios::binary);
      boost::archive::binary_oarchive oa(ofs);
      oa << strVocabularyName;
      oa << strVocabularyChecksum;
      oa << mpAtlas;
      cout << "End to write save binary file" << endl;
    }
  }
}

bool System::LoadAtlas(int type) {
  string strFileVoc, strVocChecksum;

  const string mStrLoadAtlasFromFile = settings_->atlasLoadFile();
  const string vocabularyFilePath = settings_->strVocFile_;
  bool isRead = false;

  string pathLoadFileName = "./";
  pathLoadFileName = pathLoadFileName.append(mStrLoadAtlasFromFile);
  pathLoadFileName = pathLoadFileName.append(".osa");

  if (type == TEXT_FILE) {
    // File text
    cout << "Starting to read the save text file " << endl;
    std::ifstream ifs(pathLoadFileName, std::ios::binary);
    if (!ifs.good()) {
      cout << "Load file not found" << endl;
      return false;
    }
    boost::archive::text_iarchive ia(ifs);
    ia >> strFileVoc;
    ia >> strVocChecksum;
    ia >> mpAtlas;

    cout << "Finished loading the saved text file " << endl;
    isRead = true;
  } else if (type == BINARY_FILE) {
    // File binary
    cout << "Starting to read the save binary file" << endl;
    std::ifstream ifs(pathLoadFileName, std::ios::binary);
    if (!ifs.good()) {
      cout << "Load file not found" << endl;
      return false;
    }
    boost::archive::binary_iarchive ia(ifs);
    ia >> strFileVoc;
    ia >> strVocChecksum;
    ia >> mpAtlas;

    cout << "Finished loading the saved binary file" << endl;
    isRead = true;
  }

  if (!mpAtlas) {
    throw std::runtime_error("mpAtlas not initialized when it should be");
  }

  if (isRead) {
    // Check if the vocabulary is the same
    string strInputVocabularyChecksum =
        CalculateCheckSum(vocabularyFilePath, TEXT_FILE);

    if (strInputVocabularyChecksum.compare(strVocChecksum) != 0) {
      cout << "The vocabulary load isn't the same which the load session was "
              "created "
           << endl;
      cout << "-Vocabulary name: " << strFileVoc << endl;
      return false;  // Both are differents
    }

    mpAtlas->SetKeyFrameDababase(mpKeyFrameDatabase);
    mpAtlas->SetORBVocabulary(mpVocabulary);
    mpAtlas->PostLoad();

    return true;
  }
  return false;
}

string System::CalculateCheckSum(string filename, int type) {
  string checksum = "";

  unsigned char c[MD5_DIGEST_LENGTH];

  std::ios_base::openmode flags = std::ios::in;
  if (type == BINARY_FILE)  // Binary file
    flags = std::ios::in | std::ios::binary;

  ifstream f(filename.c_str(), flags);
  if (!f.is_open()) {
    cout << "[E] Unable to open the in file " << filename << " for Md5 hash."
         << endl;
    return checksum;
  }

  MD5_CTX md5Context;
  char buffer[1024];

  MD5_Init(&md5Context);
  while (int count = f.readsome(buffer, sizeof(buffer))) {
    MD5_Update(&md5Context, buffer, count);
  }

  f.close();

  MD5_Final(c, &md5Context);

  for (int i = 0; i < MD5_DIGEST_LENGTH; i++) {
    char aux[10];
    snprintf(aux, sizeof(aux), "%02x", c[i]);
    checksum = checksum + aux;
  }

  return checksum;
}

}  // namespace ORB_SLAM3
