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

#ifndef FRAME_H
#define FRAME_H

#include<vector>

#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "KeyFrame.h"
#include "ORBextractor.h"

#include <opencv2/opencv.hpp>

namespace ORB_SLAM2
{
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64

class MapPoint;
class KeyFrame;

class Frame
{
public:
    Frame();

    // Copy constructor.
    /**
     * @brief mLastFrame = Frame(mCurrentFrame) deep copy
     * @param frame
     */
    Frame(const Frame &frame);

    // Constructor for stereo cameras.
    /**
     * @brief Frame construct for stereo
     * @param imLeft
     * @param imRight
     * @param timeStamp
     * @param extractorLeft
     * @param extractorRight
     * @param voc
     * @param K
     * @param distCoef
     * @param bf                // baseline * f
     * @param thDepth           // Threshold of far depth and near depth
     */
    Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    // Constructor for RGB-D cameras.
    /**
     * @brief Frame construct for RGB-D
     * @param imGray
     * @param imDepth
     * @param timeStamp
     * @param extractor
     * @param voc
     * @param K
     * @param distCoef
     * @param bf
     * @param thDepth
     */
    Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    // Constructor for Monocular cameras.
    /**
     * @brief Frame construct for Monocular camera
     * @param imGray
     * @param timeStamp
     * @param extractor
     * @param voc
     * @param K
     * @param distCoef
     * @param bf
     * @param thDepth
     */
    Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

    // Extract ORB on the image. 0 for left image and 1 for right image.
    /**
     * @brief Extract ORB feature
     * @param flag   left or right image
     * @param im
     */
    void ExtractORB(int flag, const cv::Mat &im);

    // Compute Bag of Words representation.
    void ComputeBoW();

    // Set the camera pose.
    void SetPose(cv::Mat Tcw);

    // Computes rotation, translation and camera center matrices from the camera pose.
    void UpdatePoseMatrices();

    // Returns the camera center.
    inline cv::Mat GetCameraCenter(){
        return mOw.clone();
    }

    // Returns inverse of rotation
    inline cv::Mat GetRotationInverse(){
        return mRwc.clone();
    }

    // Check if a MapPoint is in the frustum of the camera
    // and fill variables of the MapPoint to be used by the tracking
    /**
     * @brief Check if a MapPoint is in the frustum of the camera
     * @details Step 1 get the point's global coordinate
     * Step 2 Check if this MapPoint's depth + or - in this camera coordinate
     * Step 3 Project the point to uv coordinate, check if it is valid
     * Step 4 Compute the distance between the point and camera light, check the scale
     * Step 5 Compute the cos between FOV and normals, check if is in threshold
     * Step 6 Predict a scal
     * Step 7 Record params
     * @param pMP
     * @param viewingCosLimit
     * @return If this point is in the FOV
     */
    bool isInFrustum(MapPoint* pMP, float viewingCosLimit);

    // Compute the cell of a keypoint (return false if outside the grid)
    /**
     * @brief Compute the point's grid coordinate, and storage it in nGridPosX,nGridPosY. Return false if could not find
     * @param kp
     * @param posX
     * @param posY
     * @return
     */
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

    /**
     * @brief Find keypoints in the setting area, pyramid[minLevel,maxLevel]
     * @param x
     * @param y
     * @param r
     * @param minLevel
     * @param maxLevel
     * @return  vector<size_t>   candidate keypoint id
     */
    vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;

    // Search a match for each keypoint in the left image to a keypoint in the right image.
    // If there is a match, depth is computed and the right coordinate associated to the left keypoint is stored.
    /**
     * @brief Match the keypoint of stereo image. After the match, update Frame::mvuRight & Frame::mvDepth.
     */
    void ComputeStereoMatches();

    // Associate a "right" coordinate to a keypoint if there is valid depth in the depthmap.
    /**
     * @brief Calculate image right from RGB-D
     * @param imDepth
     */
    void ComputeStereoFromRGBD(const cv::Mat &imDepth);

    // Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
    /**
     * @brief Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates
     * @param i
     * @return
     */
    cv::Mat UnprojectStereo(const int &i);

public:
    // Vocabulary used for relocalization.
    ORBVocabulary* mpORBvocabulary;

    // Feature extractor. The right is used only in the stereo case.
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;

    // Frame timestamp.
    double mTimeStamp;

    // Calibration matrix and OpenCV distortion parameters.
    // Camera intrinsics and distortion parameters
    cv::Mat mK;
    static float fx;
    static float fy;
    static float cx;
    static float cy;
    static float invfx;
    static float invfy;
    cv::Mat mDistCoef;

    // Stereo baseline multiplied by fx.
    float mbf;

    // Stereo baseline in meters.
    float mb;

    // Threshold close/far points. Close points are inserted from 1 view.
    // Far points are inserted as in the monocular case from 2 views.
    float mThDepth;

    // Number of KeyPoints.
    int N;

    // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
    // In the stereo case, mvKeysUn is redundant as images must be rectified.
    // In the RGB-D case, RGB images can be distorted.
    std::vector<cv::KeyPoint> mvKeys, mvKeysRight; // Keypoint from origin (distorted) left and right image
    std::vector<cv::KeyPoint> mvKeysUn;            // Undistorted keypoint

    // Corresponding stereo coordinate and depth for each keypoint.
    // "Monocular" keypoints have a negative value.
    std::vector<float> mvuRight;                  // U of right image, calculate from left image
    std::vector<float> mvDepth;

    // Bag of Words Vector structures.
    DBoW2::BowVector mBowVec;          // storage std::map<WordID, WordValue>   WordValue means word's weight
    // Storage std::map<NodeId, std::vector<unsigned int>>   std::vector<unsigned int> storage all keypoint index in this node
    DBoW2::FeatureVector mFeatVec;

    // ORB descriptor, each row associated to a keypoint.
    cv::Mat mDescriptors, mDescriptorsRight;

    // MapPoints associated to keypoints, NULL pointer if no association.
    std::vector<MapPoint*> mvpMapPoints;

    // Flag to identify outlier associations.
    // Outlier flag
    std::vector<bool> mvbOutlier;

    // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
    // Grid width and height inv. It means tha coordinate*the inv can get grid index
    static float mfGridElementWidthInv;
    static float mfGridElementHeightInv;
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];  // The grid

    // Camera pose.
    cv::Mat mTcw;

    // Current and Next Frame id.
    static long unsigned int nNextId; // Next frame id
    long unsigned int mnId;           // Current frame id

    // Reference Keyframe.
    KeyFrame* mpReferenceKF;

    // Scale pyramid info.
    int mnScaleLevels;
    float mfScaleFactor;
    float mfLogScaleFactor;           // Log(Factor) use for predict the mappoint's scale
    vector<float> mvScaleFactors;
    vector<float> mvInvScaleFactors;
    vector<float> mvLevelSigma2;
    vector<float> mvInvLevelSigma2;

    // Undistorted Image Bounds (computed once).
    static float mnMinX;
    static float mnMaxX;
    static float mnMinY;
    static float mnMaxY;

    static bool mbInitialComputations;     // If the frame is init


private:

    // Undistort keypoints given OpenCV distortion parameters.
    // Only for the RGB-D case. Stereo must be already rectified!
    // (called in the constructor).
    /// The result will storage in mvKeysUn
    void UndistortKeyPoints();

    // Computes image bounds for the undistorted image (called in the constructor).
    void ComputeImageBounds(const cv::Mat &imLeft);

    // Assign keypoints to the grid for speed up feature matching (called in the constructor).
    void AssignFeaturesToGrid();

    // Rotation, translation and camera center
    cv::Mat mRcw;   // Rotation from world to camera
    cv::Mat mtcw;   // Translation from world to camera
    cv::Mat mRwc;   // Rotation from camera to world
    cv::Mat mOw; //==mtwc Translation from camera to world
};

}// namespace ORB_SLAM

#endif // FRAME_H
