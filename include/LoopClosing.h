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

#include <g2o/types/sim3/types_seven_dof_expmap.h>

#include <boost/algorithm/string.hpp>
#include <functional>
#include <list>
#include <memory>
#include <mutex>
#include <queue>
#include <set>
#include <string>
#include <thread>
#include <utility>
#include <vector>

// [VPR-BRIDGE] ROS2 headers for the external VPR interface.
// ORB-SLAM3 is a plain CMake project — we use the ROS2 client library directly
// without wrapping in an ament package.
// Prerequisites:
//   1. colcon build orbslam3_vpr_msgs first so generated headers exist
//   2. source ~/Documents/odometry_ws/install/setup.bash before cmake
#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <orbslam3_vpr_msgs/msg/key_frame_query.hpp>
#include <orbslam3_vpr_msgs/msg/vpr_candidate.hpp>
// [VPR-BRIDGE] end

#include "Atlas.h"
#include "KeyFrame.h"
#include "KeyFrameDatabase.h"
#include "LocalMapping.h"
#include "ORBVocabulary.h"
#include "Tracking.h"

namespace ORB_SLAM3 {

class Tracking;
class LocalMapping;
class KeyFrameDatabase;
class Map;

class LoopClosing : public enable_shared_from_this<LoopClosing> {
 public:
  typedef pair<set<std::shared_ptr<KeyFrame> >, int> ConsistentGroup;
  typedef map<std::shared_ptr<KeyFrame>, g2o::Sim3,
              std::less<std::shared_ptr<KeyFrame> >,
              Eigen::aligned_allocator<
                  std::pair<const std::shared_ptr<KeyFrame>, g2o::Sim3> > >
      KeyFrameAndPose;

 public:
  LoopClosing() = delete;
  LoopClosing(const LoopClosing &) = delete;

  LoopClosing(const std::shared_ptr<Atlas> &pAtlas,
              const std::shared_ptr<KeyFrameDatabase> &pDB,
              const std::shared_ptr<ORBVocabulary> &pVoc, const bool bFixScale,
              const bool bActiveLC);

  // [VPR-BRIDGE] Destructor cancels the rclcpp executor and joins the spin
  // thread cleanly. Must be defined in LoopClosing.cc.
  ~LoopClosing();
  // [VPR-BRIDGE] end

  void SetTracker(const std::shared_ptr<Tracking> &pTracker);

  void SetLocalMapper(const std::shared_ptr<LocalMapping> &pLocalMapper);

  // Main function
  void Run();

  void InsertKeyFrame(const std::shared_ptr<KeyFrame> &pKF);

  void RequestReset();
  void RequestResetActiveMap(const std::shared_ptr<Map> &pMap);

  // This function will run in a separate thread
  void RunGlobalBundleAdjustment(const std::shared_ptr<Map> &pActiveMap,
                                 unsigned long nLoopKF);

  bool isRunningGBA() {
    unique_lock<std::mutex> lock(mMutexGBA);
    return mbRunningGBA;
  }
  bool isFinishedGBA() {
    unique_lock<std::mutex> lock(mMutexGBA);
    return mbFinishedGBA;
  }

  void RequestFinish();

  bool isFinished();

  // \amm Unused?
  std::shared_ptr<Viewer> mpViewer;

#ifdef REGISTER_TIMES

  vector<double> vdDataQuery_ms;
  vector<double> vdEstSim3_ms;
  vector<double> vdPRTotal_ms;

  vector<double> vdMergeMaps_ms;
  vector<double> vdWeldingBA_ms;
  vector<double> vdMergeOptEss_ms;
  vector<double> vdMergeTotal_ms;
  vector<int> vnMergeKFs;
  vector<int> vnMergeMPs;
  int nMerges;

  vector<double> vdLoopFusion_ms;
  vector<double> vdLoopOptEss_ms;
  vector<double> vdLoopTotal_ms;
  vector<int> vnLoopKFs;
  int nLoop;

  vector<double> vdGBA_ms;
  vector<double> vdUpdateMap_ms;
  vector<double> vdFGBATotal_ms;
  vector<int> vnGBAKFs;
  vector<int> vnGBAMPs;
  int nFGBA_exec;
  int nFGBA_abort;

#endif

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

 protected:
  bool CheckNewKeyFrames();

  // Methods to implement the new place recognition algorithm
  bool NewDetectCommonRegions();
  bool DetectAndReffineSim3FromLastKF(
      const std::shared_ptr<KeyFrame> &pCurrentKF,
      std::shared_ptr<KeyFrame> &pMatchedKF, g2o::Sim3 &gScw,
      int &nNumProjMatches, std::vector<MapPoint *> &vpMPs,
      std::vector<MapPoint *> &vpMatchedMPs);
  bool DetectCommonRegionsFromBoW(
      std::vector<std::shared_ptr<KeyFrame> > &vpBowCand,
      std::shared_ptr<KeyFrame> &pMatchedKF,
      std::shared_ptr<KeyFrame> &pLastCurrentKF, g2o::Sim3 &g2oScw,
      int &nNumCoincidences, std::vector<MapPoint *> &vpMPs,
      std::vector<MapPoint *> &vpMatchedMPs);
  bool DetectCommonRegionsFromLastKF(
      const std::shared_ptr<KeyFrame> &pCurrentKF,
      const std::shared_ptr<KeyFrame> &pMatchedKF, g2o::Sim3 &gScw,
      int &nNumProjMatches, std::vector<MapPoint *> &vpMPs,
      std::vector<MapPoint *> &vpMatchedMPs);
  int FindMatchesByProjection(const std::shared_ptr<KeyFrame> &pCurrentKF,
                              const std::shared_ptr<KeyFrame> &pMatchedKFw,
                              g2o::Sim3 &g2oScw,
                              set<MapPoint *> &spMatchedMPinOrigin,
                              vector<MapPoint *> &vpMapPoints,
                              vector<MapPoint *> &vpMatchedMapPoints);

  void SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap,
                     vector<MapPoint *> &vpMapPoints);
  void SearchAndFuse(const vector<std::shared_ptr<KeyFrame> > &vConectedKFs,
                     vector<MapPoint *> &vpMapPoints);

  void CorrectLoop();

  void MergeLocal();
  void MergeLocal2();

  void CheckObservations(set<std::shared_ptr<KeyFrame> > &spKFsMap1,
                         set<std::shared_ptr<KeyFrame> > &spKFsMap2);

  // [VPR-BRIDGE] ----------------------------------------------------------
  // Called once at the end of the constructor. Creates mpVPRNode, sets up
  // the publisher and subscription, then launches mVPRSpinThread.
  // Requires rclcpp::init() to have been called beforehand (done in System.cc).
  void InitVPRBridge();

  // Publish a KeyFrame's grayscale image to the Python VPR node.
  // Called from NewDetectCommonRegions() in place of the DBoW2 query.
  // If mImGray is empty the publish is skipped with a warning.
  void PublishKeyFrameForVPR(const std::shared_ptr<KeyFrame> &pKF);

  // Drain the incoming VPRCandidate queue for the current KF.
  // Same-map matches → vpLoopCand; different-map matches → vpMergeCand.
  // Returns true if at least one valid candidate was found in either list.
  // Entries for other KFs are returned to the queue for future ticks.
  bool FetchVPRCandidates(
      std::vector<std::shared_ptr<KeyFrame>> &vpLoopCand,
      std::vector<std::shared_ptr<KeyFrame>> &vpMergeCand);

  // ROS2 subscription callback — runs on mVPRSpinThread.
  // Pushes incoming messages into mvVPRCandidateQueue under mMutexVPRQueue.
  void VPRCandidateCallback(
      const orbslam3_vpr_msgs::msg::VPRCandidate::SharedPtr msg);
  // [VPR-BRIDGE] end ------------------------------------------------------

  void ResetIfRequested();
  bool mbResetRequested;
  bool mbResetActiveMapRequested;
  std::shared_ptr<Map> mpMapToReset;
  std::mutex mMutexReset;

  bool CheckFinish();
  void SetFinish();
  bool mbFinishRequested;
  bool mbFinished;
  std::mutex mMutexFinish;

  std::shared_ptr<Atlas> mpAtlas;
  std::shared_ptr<Tracking> mpTracker;

  std::shared_ptr<KeyFrameDatabase> mpKeyFrameDB;
  std::shared_ptr<ORBVocabulary> mpORBVocabulary;

  std::shared_ptr<LocalMapping> mpLocalMapper;

  std::list<std::shared_ptr<KeyFrame> > mlpLoopKeyFrameQueue;

  std::mutex mMutexLoopQueue;

  // Loop detector parameters
  float mnCovisibilityConsistencyTh;

  // Loop detector variables
  std::shared_ptr<KeyFrame> mpCurrentKF;
  std::shared_ptr<KeyFrame> mpLastCurrentKF;
  std::shared_ptr<KeyFrame> mpMatchedKF;
  std::vector<ConsistentGroup> mvConsistentGroups;
  std::vector<std::shared_ptr<KeyFrame> > mvpEnoughConsistentCandidates;
  std::vector<std::shared_ptr<KeyFrame> > mvpCurrentConnectedKFs;
  std::vector<MapPoint *> mvpCurrentMatchedPoints;
  std::vector<MapPoint *> mvpLoopMapPoints;
  cv::Mat mScw;
  g2o::Sim3 mg2oScw;

  //-------
  std::shared_ptr<Map> mpLastMap;

  bool mbLoopDetected;
  int mnLoopNumCoincidences;
  int mnLoopNumNotFound;
  std::shared_ptr<KeyFrame> mpLoopLastCurrentKF;
  g2o::Sim3 mg2oLoopSlw;
  g2o::Sim3 mg2oLoopScw;
  std::shared_ptr<KeyFrame> mpLoopMatchedKF;
  std::vector<MapPoint *> mvpLoopMPs;
  std::vector<MapPoint *> mvpLoopMatchedMPs;
  bool mbMergeDetected;
  int mnMergeNumCoincidences;
  int mnMergeNumNotFound;
  std::shared_ptr<KeyFrame> mpMergeLastCurrentKF;
  g2o::Sim3 mg2oMergeSlw;
  g2o::Sim3 mg2oMergeSmw;
  g2o::Sim3 mg2oMergeScw;
  std::shared_ptr<KeyFrame> mpMergeMatchedKF;
  std::vector<MapPoint *> mvpMergeMPs;
  std::vector<MapPoint *> mvpMergeMatchedMPs;
  std::vector<std::shared_ptr<KeyFrame> > mvpMergeConnectedKFs;

  g2o::Sim3 mSold_new;
  //-------

  long unsigned int mLastLoopKFid;

  // Variables related to Global Bundle Adjustment
  bool mbRunningGBA;
  bool mbFinishedGBA;
  bool mbStopGBA;
  std::mutex mMutexGBA;
  std::thread *mpThreadGBA;

  // Fix scale in the stereo/RGB-D case
  bool mbFixScale;

  int mnFullBAIdx;

  vector<double> vdPR_CurrentTime;
  vector<double> vdPR_MatchedTime;
  vector<int> vnPR_TypeRecogn;

  // DEBUG
  string mstrFolderSubTraj;
  int mnNumCorrection;
  int mnCorrectionGBA;

  // To (de)activate LC
  bool mbActiveLC = true;

  // [VPR-BRIDGE] ROS2 node, publisher, subscription and executor ----------
  //
  // mpVPRNode       — standalone rclcpp::Node owned by LoopClosing.
  // mVPRImagePub    — publishes KeyFrameQuery to /vpr/keyframe.
  // mVPRCandidateSub— receives VPRCandidate from /vpr/loop_candidates.
  // mVPRExecutor    — SingleThreadedExecutor driving mpVPRNode callbacks.
  // mVPRSpinThread  — dedicated thread that calls mVPRExecutor.spin().
  //
  // VPRCandidate    — plain struct used as the inter-thread hand-off type.
  //                   Keeps the queue independent of the ROS2 message type
  //                   so the LC thread never touches ROS2 memory directly.
  //
  // mvVPRCandidateQueue / mMutexVPRQueue
  //                 — thread-safe buffer between the rclcpp callback thread
  //                   and the LoopClosing thread.
  rclcpp::Node::SharedPtr mpVPRNode;
  rclcpp::Publisher<orbslam3_vpr_msgs::msg::KeyFrameQuery>::SharedPtr
      mVPRImagePub;
  rclcpp::Subscription<orbslam3_vpr_msgs::msg::VPRCandidate>::SharedPtr
      mVPRCandidateSub;
  rclcpp::executors::SingleThreadedExecutor mVPRExecutor;
  std::thread mVPRSpinThread;

  struct VPRCandidate {
    long unsigned int queryKFId;
    long unsigned int matchedKFId;
    float             score;
  };
  std::queue<VPRCandidate> mvVPRCandidateQueue;
  std::mutex               mMutexVPRQueue;
  // [VPR-BRIDGE] end ------------------------------------------------------

#ifdef REGISTER_LOOP
  string mstrFolderLoop;
#endif
};

}  // namespace ORB_SLAM3
