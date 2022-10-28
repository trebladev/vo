/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM3>
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


#ifndef ORBMATCHER_H
#define ORBMATCHER_H

#include<vector>
#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"MapPoint.h"
#include"KeyFrame.h"
#include"Frame.h"

namespace ORB_SLAM2 {

class ORBmatcher {
 public:

  /**
   * Constructor
   * @param nnratio ratio of the best and the second score
   * @param checkOri check orientation
   */
  ORBmatcher(float nnratio = 0.6, bool checkOri = true);

  // Computes the Hamming distance between two ORB descriptors
  /**
   * Computes the Hamming distance between two ORB descriptors
   * @param a
   * @param b
   * @return
   */
  static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);

  // Search matches between Frame keypoints and projected MapPoints. Returns number of matches
  // Used to track the local map (Tracking)
  /**
   * @brief project mappoint to current frame, Used to track the local map
   * Step 1 Traverse the local map point
   * Step 2 Set slide window's size. The size depend on FOV, when FOV is much different with average FOV, with small size
   * Step 3 Search the match point in windows
   * Step 4 Find the best match point and the better matcher point
   * Step 5 Screening the best match point
   * @param F                   Current frame
   * @param vpMapPoints         Local map points
   * @param th                  Range of search
   * @return                    Number of succeed match point
   */
  int SearchByProjection(Frame &F, const std::vector<MapPoint *> &vpMapPoints, const float th = 3);

  // Project MapPoints tracked in last frame into the current frame and search matches.
  // Used to track from previous frame (Tracking)
  /**
   * @brief Project last frame's mappoint to current frame, and search match point. Track from previous frame
   * Step 1 Establish a rotation histogram to detect rotation consistency
   * Step 2 Calculate the translate from current frame to last frame
   * Step 3 Project Mappoint to camera pixel plane
   * Step 4 Set search range from camera direction
   * Step 5 Traverse candidate points, find the point which has smaller distance
   * Step 6 Compute the histogram about rotation angle error
   * Step 7 Check orientation, delete the error match point
   * @param CurrentFrame
   * @param LastFrame
   * @param th
   * @param bMono
   * @return
   */
  int SearchByProjection(Frame &CurrentFrame, const Frame &LastFrame, const float th, const bool bMono);

  // Project MapPoints seen in KeyFrame into the Frame and search matches.
  // Used in relocalisation (Tracking)
  /**
   * @brief Project keypoint in keyframe to current frame
   * @param CurrentFrame  Current frame
   * @param pKF           Keyframe
   * @param sAlreadyFound Mappoint has founded
   * @param th            Range of windows
   * @param ORBdist       Threshold of minimum distance between to descriptors
   * @return
   */
  int SearchByProjection(Frame &CurrentFrame,
                         KeyFrame *pKF,
                         const std::set<MapPoint *> &sAlreadyFound,
                         const float th,
                         const int ORBdist);

  // Project MapPoints using a Similarity Transformation and search matches.
  // Used in loop detection (Loop Closing)
  /**
   * @brief According to Sim3, project vpPoints to pKF, and search for range
   * @details March the point's descriptor with keyframe keypoint, update the vpMatched
   * @param pKF        Keyframe
   * @param Scw
   * @param vpPoints   Mappoint
   * @param vpMatched  Matched point
   * @param th         Range of windows
   * @return           Number of matched points
   */
  int SearchByProjection(KeyFrame *pKF,
                         cv::Mat Scw,
                         const std::vector<MapPoint *> &vpPoints,
                         std::vector<MapPoint *> &vpMatched,
                         int th);

  // Search matches between MapPoints in a KeyFrame and ORB in a Frame.
  // Brute force constrained to ORB that belong to the same vocabulary node (at a certain level)
  // Used in Relocalisation and Loop Detection
  /**
   * @brief Tracking keypoint with bag of words
   * Step 1 Get keypoint belong to same node
   * Step 2 Traverse keypoints in key frame which belong to this node
   * Step 3 Traverse keypoints in current frame which belong to this node
   * Step 4 Delete the wrong match by threshold and angle
   * Step 5 Delete the wrong match by orientation
   * @param pKF
   * @param F
   * @param vpMapPointMatches
   * @return
   */
  int SearchByBoW(KeyFrame *pKF, Frame &F, std::vector<MapPoint *> &vpMapPointMatches);
  int SearchByBoW(KeyFrame *pKF1, KeyFrame *pKF2, std::vector<MapPoint *> &vpMatches12);

  // Matching for the Map Initialization (only used in the monocular case)
  /**
   * @brief Match the current frame and key frame's keypoint in monocular init
   * Step 1 Construct the rotation histogram
   * Step 2 Search candidate keypoints in F2 with redius
   * Step 3 Traverse all candidate keypoint, find the best and second
   * Step 4 Check the result, delete the repeat matching
   * Step 5 Calculate the histogram about angle error
   * Step 6
   * Step 7 Save the match
   * @param F1
   * @param F2
   * @param vbPrevMatched
   * @param vnMatches12
   * @param windowSize
   * @return
   */
  int SearchForInitialization(Frame &F1,
                              Frame &F2,
                              std::vector<cv::Point2f> &vbPrevMatched,
                              std::vector<int> &vnMatches12,
                              int windowSize = 10);

  // Matching to triangulate new MapPoints. Check Epipolar Constraint.
  /**
   * @brief Use fundamental matrix, compute 3d point
   * @param pKF1           Keyframe 1
   * @param pKF2           Keyframe 2
   * @param F12            Fundamental matrix
   * @param vMatchedPairs  Matched point pairs
   * @param bOnlyStereo    Whether have two pictures
   * @return
   */
  int SearchForTriangulation(KeyFrame *pKF1, KeyFrame *pKF2, cv::Mat F12,
                             std::vector<pair<size_t, size_t> > &vMatchedPairs, const bool bOnlyStereo);

  // Search matches between MapPoints seen in KF1 and KF2 transforming by a Sim3 [s12*R12|t12]
  // In the stereo and RGB-D case, s12=1
  /**
   * @brief Determined the scope of Keyframe's keypoint in Keyframe 2
   * @param pKF1
   * @param pKF2
   * @param vpMatches12
   * @param s12
   * @param R12
   * @param t12
   * @param th
   * @return
   */
  int SearchBySim3(KeyFrame *pKF1,
                   KeyFrame *pKF2,
                   std::vector<MapPoint *> &vpMatches12,
                   const float &s12,
                   const cv::Mat &R12,
                   const cv::Mat &t12,
                   const float th);

  // Project MapPoints into KeyFrame and search for duplicated MapPoints.
  /**
   * @brief Project Mappoint to Keyframe, it can be used in update keypoint
   * @param pKF
   * @param vpMapPoints
   * @param th
   * @return
   */
  int Fuse(KeyFrame *pKF, const vector<MapPoint *> &vpMapPoints, const float th = 3.0);

  // Project MapPoints into KeyFrame using a given Sim3 and search for duplicated MapPoints.
  /**
   * @brief This function is a sub-function of upper
   * @param pKF
   * @param Scw
   * @param vpPoints
   * @param th
   * @param vpReplacePoint
   * @return
   */
  int Fuse(KeyFrame *pKF,
           cv::Mat Scw,
           const std::vector<MapPoint *> &vpPoints,
           float th,
           vector<MapPoint *> &vpReplacePoint);

 public:

  static const int TH_LOW;        // Low threshold of match descriptor
  static const int TH_HIGH;       // High threshold of match descriptor
  static const int HISTO_LENGTH;  // Threshold of histogram length

 protected:

  bool CheckDistEpipolarLine(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, const cv::Mat &F12, const KeyFrame *pKF);

  /**
   * @brief Determine the windows size of search match point
   * @param viewCos
   * @return
   */
  float RadiusByViewingCos(const float &viewCos);

  /**
   * @brief Choose the three bin which has most rotation angle in histogram
   * @param histo
   * @param L
   * @param ind1
   * @param ind2
   * @param ind3
   */
  void ComputeThreeMaxima(std::vector<int> *histo, const int L, int &ind1, int &ind2, int &ind3);

  float mfNNratio;          // The ratio of best and the second
  bool mbCheckOrientation;  // Whether check the keypoint rotation
};

}// namespace ORB_SLAM

#endif // ORBMATCHER_H
