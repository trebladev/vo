//
// Created by xuan on 10/25/22.
//

#ifndef VO_INCLUDE_FRAME_H_
#define VO_INCLUDE_FRAME_H_

#include <vector>
#include <ORBextractor.h>
#include "ORBVocabulary.h"
#include <opencv2/opencv.hpp>

namespace ORB_SLAM2
{

#define FRAME_GRID_ROWS 48

#define FRAME_GRID_COLS 64

class Frame
{
 public:

  Frame();

  Frame(const Frame &frame);

  /**
   * This Constructor is for stereo camera
   * @param imLeft
   * @param imRight
   * @param timeStamp
   * @param extractorLeft
   * @param extractorRight
   * @param voc
   * @param K
   * @param distCoef
   * @param bf
   * @param thDepth
   */
  Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);

  /**
   * This Constructor is for RGBD camera
   * @param imGray
   * @param imDepth
   * @param timeStamp
   * @param extractor
   * @param voc
   * @param K
   * @param destCoef
   * @param bf
   * @param thDepth
   */
  Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor, ORBVocabulary* voc, cv::Mat &K, cv::Mat &destCoef, const float &bf, const float &thDepth);

  /**
   * This Constructor is for Monocular
   * @param imGray
   * @param timeStamp
   * @param extractor
   * @param voc
   * @param K
   * @param destCoef
   * @param bf
   * @param thDepth
   */
  Frame(const cv::Mat &imGray, const double &timeStamp, ORBextractor* extractor, ORBVocabulary* voc, cv::Mat &K, cv::Mat &destCoef, const float &bf, const float &thDepth);

  /**
   * @brief extractor ORB feature, storage Keypoint in mvKeys desc in Descriptors
   * @param flag left image or right image
   * @param im   the image
   */
  void ExtractORB(int flag, const cv::Mat &im);

  /**
   * @brief compute bag of words
   */
  void ComputeBoW();

  /**
   * @brief use Tcw to update mTcw
   * @param Tcw
   */
  void SetPose(cv::Mat Tcw);

  /**
   * @brief according to camera pose, compute camera's rotation, translation and camera center matrices.
   */
  void UpdatePoseMatrices();

 public:

  // Vocabulary
  ORBVocabulary* mpORBvocabulary;

  // Feature extractor
  ORBextractor* mpORBextractorLeft, *mpORBextractorRight;

  // Frame timestamp
  double mTimeStamp;

  cv::Mat mK;

  static float fx;
  static float fy;
  static float cx;
  static float cy;
  static float invfx;
  static float invfy;

  // matrix of distort
  cv::Mat mDistCoef;

  // Stereo baseline multiplied by fx
  float mbf;

  // Stereo baseline in meters
  float mb;

  // Threshold close/far points. Close points are inserted from 1 view
  // Far points are inserts as in the monocular case from 2 views
  float mThDepth;

  // Number of KeyPoints
  int N;

  // Vector of keypoints (original for visualization) and undistorted (actually used by the system)
  // In the stereo case, mvKeysUn is redundant as images must be rectified
  // In the RGBD case, RGB images can be distorted

  // keypoint from left
  std::vector<cv::KeyPoint> mvKeys;
  // keypoint from right image
  std::vector<cv::KeyPoint> mvKeysRight;
  // keypoint undistorted
  std::vector<cv::KeyPoint> mvKeyUn;

  // Corresponding stereo coordinate and depth for each keypoint
  // "Monocular" keypoints have a negative value
  std::vector<float> mvuRight;  // keypoint in right image's u
  std::vector<float> mvDepth;

  // Bag of Words Vector structures.
  // std::map<WordId, WordValue>
  // WordId means the word's ID in bag, WordValue means word's weight
  DBoW2::BowVector mBowVec;
  // std::map<NodeId, std::vector<unsigned int>
  // NodeId means node's Id, the vector means the idx of keypoint in this Node
  DBoW2::FeatureVector mFeatVec;

  // ORB descriptor, each row associated to a keypoint
  cv::Mat mDescriptors, mDescriptorsRight;

  // MapPoints associated to keypoints, NULL pointer if no association
  // TODO WAITING FOR CLASS MAPPOINTS

  // Flag to identify outliwe associations
  std::vector<bool> mvbOutlier;

  // Keypoints are assigned to cells in a grid to reduce matching complexity when projection mappoints
  static float mfGridElementWidthInv;   // coordinate time mfGridElementWidthInv and mfGridElementHeightInv could sure the grid
  static float mfGridElementHeightInv;

  // storage the keypoint id in grid
  std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

  // camera pose
  cv::Mat mTcw;

  // Current and Next Frame id
  static long unsigned int nNextId;
  long unsigned int mnId;

  // Reference Keyframe
  // TODO WAITING FOR CLASS KEYFRAME

  // Scale pyramid info
  int mnScaleLevels;                  ///<图像金字塔的层数
  float mfScaleFactor;                ///<图像金字塔的尺度因子
  float mfLogScaleFactor;             ///<图像金字塔的尺度因子的对数值，用于仿照特征点尺度预测地图点的尺度

  vector<float> mvScaleFactors;		///<图像金字塔每一层的缩放因子
  vector<float> mvInvScaleFactors;	///<以及上面的这个变量的倒数
  vector<float> mvLevelSigma2;		///@todo 目前在frame.c中没有用到，无法下定论
  vector<float> mvInvLevelSigma2;		///<上面变量的倒数


  // Undistorted Image Bounds (computed once).
  static float mnMinX;
  static float mnMaxX;
  static float mnMinY;
  static float mnMaxY;

  /**
   * @brief 一个标志，标记是否已经进行了这些初始化计算
   * @note 由于第一帧以及SLAM系统进行重新校正后的第一帧会有一些特殊的初始化处理操作，所以这里设置了这个变量. \n
   * 如果这个标志被置位，说明再下一帧的帧构造函数中要进行这个“特殊的初始化操作”，如果没有被置位则不用。
  */
  // A flag id init
  static bool mbInitialComputations;

private:

  // Undistort keypoints given OpenCV distortion parameters.
  // Only for the RGB-D case. Stereo must be already rectified!
  // (called in the constructor).

  // result storage in mvKeyUn
  void UndistortKeyPoints();

  void ComputeImageBounds(const cv::Mat &imLeft);

  // Assign keypoints to the grid for speed up feature matching (called in the constructor).
  void AssignFeaturesToGrid();

  // Rotation, translation and camera center
  cv::Mat mRcw; ///< Rotation from world to camera
  cv::Mat mtcw; ///< Translation from world to camera
  cv::Mat mRwc; ///< Rotation from camera to world
  cv::Mat mOw;  ///< mtwc,Translation from camera to world

};
}

#endif //VO_INCLUDE_FRAME_H_
