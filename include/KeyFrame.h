/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef KEYFRAME_H
#define KEYFRAME_H

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"
#include "Frame.h"
#include "KeyFrameDatabase.h"

#include <mutex>

namespace ORB_SLAM2 {

class Map;
class MapPoint;
class Frame;
class KeyFrameDatabase;

class KeyFrame {
 public:

  /**
   * @brief Construct
   * @param F
   * @param pMap
   * @param pKFDB
   */
  KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB);

  // Pose functions
  void SetPose(const cv::Mat &Tcw);
  cv::Mat GetPose();
  cv::Mat GetPoseInverse();
  cv::Mat GetCameraCenter();
  cv::Mat GetStereoCenter();
  cv::Mat GetRotation();
  cv::Mat GetTranslation();

  // Bag of Words Representation
  /**
   * @brief Bag of Words Representation
   * @details Compute mBowVec, distribute is on level4, mFeatVec record NO.i node's keypoint ni
   * @see ProcessNewKeyFrame()
   */
  void ComputeBoW();

  // Covisibility graph functions
  /**
   * @brief Add connect between keyframe
   * @param pKF
   * @param weight
   */
  void AddConnection(KeyFrame *pKF, const int &weight);
  void EraseConnection(KeyFrame *pKF);
  void UpdateConnections();
  /**
   * @brief Sort keyframe by weight
   * @details Variable storage in mvpOrderdConectedKeyFrames and mvOrderedWeights
   */
  void UpdateBestCovisibles();
  /**
   * @brief Get keyframes which connect with current keyframe(Non sort)
   * @return
   */
  std::set<KeyFrame *> GetConnectedKeyFrames();
  /**
   * @brief Get keyframes which connect with current keyframe(Sort)
   * @return
   */
  std::vector<KeyFrame *> GetVectorCovisibleKeyFrames();
  /**
   * @brief Get top n keyframe which connect with current keyframe
   * @param N
   * @return
   */
  std::vector<KeyFrame *> GetBestCovisibilityKeyFrames(const int &N);
  /**
   * @brief Get Top n keyframe which connect with current keyframe(Sort by weight)
   * @param w
   * @return
   */
  std::vector<KeyFrame *> GetCovisiblesByWeight(const int &w);
  int GetWeight(KeyFrame *pKF);

  // Spanning tree functions
  /**
   * @brief Add keyframe for spanning tree as a child
   * @param pKF
   */
  void AddChild(KeyFrame *pKF);
  void EraseChild(KeyFrame *pKF);
  /**
   * @brief Change the current frame's parent node
   * @param pKF
   */
  void ChangeParent(KeyFrame *pKF);
  std::set<KeyFrame *> GetChilds();
  KeyFrame *GetParent();
  bool hasChild(KeyFrame *pKF);

  // Loop Edges
  /**
   * @brief Add loopclosure edge for current keyframe
   * @param pKF
   */
  void AddLoopEdge(KeyFrame *pKF);
  std::set<KeyFrame *> GetLoopEdges();

  // MapPoint observation functions
  /**
   * @brief Add Mappoint to keyframe
   * @param pMP Mappoint
   * @param idx Mappoint's index in current keyframe
   */
  void AddMapPoint(MapPoint *pMP, const size_t &idx);
  void EraseMapPointMatch(const size_t &idx);
  void EraseMapPointMatch(MapPoint *pMP);
  /**
   * @brief Replace mappoitn idx by new pMP
   * @param idx
   * @param pMP
   */
  void ReplaceMapPointMatch(const size_t &idx, MapPoint *pMP);
  std::set<MapPoint *> GetMapPoints();
  std::vector<MapPoint *> GetMapPointMatches();
  /**
   * @brief The mappoint in current keyframe which observer time more than minObs
   * @param minObs
   * @return
   */
  int TrackedMapPoints(const int &minObs);
  MapPoint *GetMapPoint(const size_t &idx);

  // KeyPoint functions
  /**
   * @brief Get keypoint's id which in a certain keypoint and redius
   * @param x  keypoint's x
   * @param y  keypoint's y
   * @param r  redius
   * @return
   */
  std::vector<size_t> GetFeaturesInArea(const float &x, const float &y, const float &r) const;
  /**
   * @brief Backprojects a keypoint into 3D world coordinate
   * @param i
   * @return
   */
  cv::Mat UnprojectStereo(int i);

  // Image
  bool IsInImage(const float &x, const float &y) const;

  // Enable/Disable bad flag changes
  void SetNotErase();
  void SetErase();

  // Set/check bad flag
  void SetBadFlag();
  bool isBad();

  // Compute Scene Depth (q=2 median). Used in monocular.
  /**
   * @brief Compute Scene MedianDepth
   * @param q
   * @return
   */
  float ComputeSceneMedianDepth(const int q);

  // Compare two weight
  static bool weightComp(int a, int b) {
    return a > b;
  }

  // Compare two keyframe's ID
  static bool lId(KeyFrame *pKF1, KeyFrame *pKF2) {
    return pKF1->mnId < pKF2->mnId;
  }


  // The following variables are accesed from only 1 thread or never change (no mutex needed).
 public:

  static long unsigned int nNextId;
  long unsigned int mnId;
  const long unsigned int mnFrameId;

  const double mTimeStamp;

  // Grid (to speed up feature matching)
  const int mnGridCols;
  const int mnGridRows;
  const float mfGridElementWidthInv;
  const float mfGridElementHeightInv;

  // Variables used by the tracking
  long unsigned int mnTrackReferenceForFrame;
  long unsigned int mnFuseTargetForKF;        // Record keyframe which fused (in local mapping thread)

  // Variables used by the local mapping
  long unsigned int mnBALocalForKF; // Keyframe's id which is processing by local ba
  long unsigned int mnBAFixedForKF; // Provide constraint information

  // Variables used by the keyframe database
  long unsigned int mnLoopQuery;  // Which keyframe is candidate of loopclose
  int mnLoopWords;                // Number for same word in two keyframe
  float mLoopScore;
  long unsigned int mnRelocQuery; // Which keyframe need relocation
  int mnRelocWords;
  float mRelocScore;

  // Variables used by loop closing
  cv::Mat mTcwGBA;     /// The pose after full ba
  cv::Mat mTcwBefGBA;  /// The pose before full ba, this pose use for update the mappoint
  long unsigned int mnBAGlobalForKF; /// Record which keyframe trigger the full ba

  // Calibration parameters
  const float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;

  // Number of KeyPoints
  const int N;

  // KeyPoints, stereo coordinate and descriptors (all associated by an index)
  const std::vector<cv::KeyPoint> mvKeys;
  const std::vector<cv::KeyPoint> mvKeysUn;
  const std::vector<float> mvuRight; // negative value for monocular points
  const std::vector<float> mvDepth; // negative value for monocular points
  const cv::Mat mDescriptors;

  //BoW
  DBoW2::BowVector mBowVec;
  DBoW2::FeatureVector mFeatVec;

  // Pose relative to parent (this is computed when bad flag is activated)
  cv::Mat mTcp;

  // Scale
  const int mnScaleLevels;
  const float mfScaleFactor;
  const float mfLogScaleFactor;
  const std::vector<float> mvScaleFactors;
  const std::vector<float> mvLevelSigma2;
  const std::vector<float> mvInvLevelSigma2;

  // Image bounds and calibration
  const int mnMinX;
  const int mnMinY;
  const int mnMaxX;
  const int mnMaxY;
  const cv::Mat mK;


  // The following variables need to be accessed trough a mutex to be thread safe.
 protected:

  // SE3 Pose and camera center
  cv::Mat Tcw; // world to camera
  cv::Mat Twc; // camera to world
  cv::Mat Ow;  // Left camera's light heart

  cv::Mat Cw; // Stereo middel point. Only for visualization

  // MapPoints associated to keypoints
  std::vector<MapPoint *> mvpMapPoints;

  // BoW
  KeyFrameDatabase *mpKeyFrameDB;
  ORBVocabulary *mpORBvocabulary;

  // Grid over the image to speed up feature matching
  std::vector<std::vector<std::vector<size_t> > > mGrid;

  std::map<KeyFrame *, int> mConnectedKeyFrameWeights;
  std::vector<KeyFrame *> mvpOrderedConnectedKeyFrames;
  std::vector<int> mvOrderedWeights;

  // Spanning Tree and Loop Edges
  // std::set will auto sort
  bool mbFirstConnection;              // If the first time build spanning Tree
  KeyFrame *mpParent;                  // The parent keyframe of current keyframe
  std::set<KeyFrame *> mspChildrens;   // The child keyframe
  std::set<KeyFrame *> mspLoopEdges;   // The keyframe which has loop with current keyframe

  // Bad flags
  bool mbNotErase;
  bool mbToBeErased;
  bool mbBad;

  float mHalfBaseline; // Only for visualization

  Map *mpMap;

  std::mutex mMutexPose;
  std::mutex mMutexConnections;
  std::mutex mMutexFeatures;
};

} //namespace ORB_SLAM

#endif // KEYFRAME_H
