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

#ifndef MAPPOINT_H
#define MAPPOINT_H

#include"KeyFrame.h"
#include"Frame.h"
#include"Map.h"

#include<opencv2/core/core.hpp>
#include<mutex>

namespace ORB_SLAM2 {

class KeyFrame;
class Map;
class Frame;

/**
 * @brief The class is of every mappoint
 */
class MapPoint {
 public:

  /**
   * @brief Construct mappoint by keyframe and coordinate
   * @details This construct function was cited in StereoInitialization(). LocalMapping::CreateNewMapPoints()
   * Monocular CreateInitialMapMonocular(). LocalMapping::CreateNeMapPoints()
   * @param Pos
   * @param pRefKF
   * @param pMap
   */
  MapPoint(const cv::Mat &Pos, KeyFrame *pRefKF, Map *pMap);
  /**
   * @brief Construct mappoint by frame and coordinate
   * @details This construct function was used in Stereo UpdateLastFrame()
   * @param Pos
   * @param pMap
   * @param pFrame
   * @param idxF
   */
  MapPoint(const cv::Mat &Pos, Map *pMap, Frame *pFrame, const int &idxF);

  void SetWorldPos(const cv::Mat &Pos);
  cv::Mat GetWorldPos();

  cv::Mat GetNormal();
  KeyFrame *GetReferenceKeyFrame();

  /**
   * @brief Get frame which observe this mappoint
   * @return Return the keyframe and the point's index in keyframe
   */
  std::map<KeyFrame *, size_t> GetObservations();

  /**
   * @brief Get observations of this mappoint
   * @return
   */
  int Observations();

  void AddObservation(KeyFrame *pKF, size_t idx);
  void EraseObservation(KeyFrame *pKF);

  int GetIndexInKeyFrame(KeyFrame *pKF);
  bool IsInKeyFrame(KeyFrame *pKF);

  void SetBadFlag();
  bool isBad();

  void Replace(MapPoint *pMP);
  MapPoint *GetReplaced();

  /**
   * @brief Increase visible
   * @details This mappoint can be observed but can not match successfully
   * @param n
   */
  void IncreaseVisible(int n = 1);
  /**
   * @brief Increase found
   * @param n
   */
  void IncreaseFound(int n = 1);

  /**
   * @brief Found / visible
   * @return
   */
  float GetFoundRatio();
  inline int GetFound() {
    return mnFound;
  }

  /**
   * @brief Compute the representative descriptors
   */
  void ComputeDistinctiveDescriptors();

  cv::Mat GetDescriptor();

  void UpdateNormalAndDepth();

  float GetMinDistanceInvariance();
  float GetMaxDistanceInvariance();
  int PredictScale(const float &currentDist, KeyFrame *pKF);
  int PredictScale(const float &currentDist, Frame *pF);

 public:
  long unsigned int mnId;            // Global ID of Mappoint
  static long unsigned int nNextId;
  long int mnFirstKFid;              // ID for keyframe which create this mappoint
  long int mnFirstFrame;             // ID for frame which create this mappoint

  // Number of observation, monocular:+1, stereo&RGB-D:+2
  int nObs;

  // Variables used by the tracking
  // This mappoint's coordinate when project into one frame X,Y and X in right image
  float mTrackProjX;
  float mTrackProjY;
  float mTrackProjXR;

  // Need track
  bool mbTrackInView;
  int mnTrackScaleLevel; // Tracking scale
  float mTrackViewCos;   // The camera FOV when tracked
  long unsigned int mnTrackReferenceForFrame;
  long unsigned int mnLastFrameSeen;

  // Variables used by local mapping
  // Marked the mappoint for avoid repeat option.
  long unsigned int mnBALocalForKF;
  long unsigned int mnFuseCandidateForKF;

  // Variables used by loop closing
  // Marked the mappoint for avoid repeat option.
  long unsigned int mnLoopPointForKF;
  long unsigned int mnCorrectedByKF;
  long unsigned int mnCorrectedReference;

  // The pose after global BA.
  cv::Mat mPosGBA;
  // Record which keyframe cause global BA.
  long unsigned int mnBAGlobalForKF;

  // A lock
  static std::mutex mGlobalMutex;

 protected:

  // Position in absolute coordinates i.e. global coordinate
  cv::Mat mWorldPos;

  // Keyframes observing the point and associated index in keyframe
  std::map<KeyFrame *, size_t> mObservations;

  // Mean viewing direction
  cv::Mat mNormalVector;

  // Best descriptor to fast matching
  cv::Mat mDescriptor;

  // Reference KeyFrame
  KeyFrame *mpRefKF;

  // Tracking counters
  int mnVisible;
  int mnFound;

  // Bad flag (we do not currently erase MapPoint from memory)
  bool mbBad;
  MapPoint *mpReplaced;

  // Scale invariance distances
  float mfMinDistance;
  float mfMaxDistance;

  Map *mpMap;

  // A lock for pose option.
  std::mutex mMutexPos;
  // A lock for feature option.
  std::mutex mMutexFeatures;
};

} //namespace ORB_SLAM

#endif // MAPPOINT_H
