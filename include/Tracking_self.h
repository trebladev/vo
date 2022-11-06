//
// Created by xuan on 11/3/22.
//

#ifndef VO_INCLUDE_TRACKING_SELF_H_
#define VO_INCLUDE_TRACKING_SELF_H_


#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include "Map.h"
#include "Frame.h"
#include "ORBVocabulary.h"
#include "KeyFrameDatabase.h"
#include "ORBextractor.h"
#include "MapDrawer.h"
#include "FrameDrawer.h"
#include "realsense2.h"
#include "ORBmatcher.h"
#include "Optimizer.h"


#include <mutex>


namespace ORB_SLAM2
{
class Map;
class FrameDrawer;

class Tracking
{
 public:

    Tracking(ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
             KeyFrameDatabase* pKFDB, const rs2::pipeline pipe, const int sensor);

    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    // 下面的函数都是对不同的传感器输入的图像进行处理(转换成为灰度图像),并且调用Tracking线程
    /**
     * @brief 处理双目输入
     *
     * @param[in] imRectLeft    左目图像
     * @param[in] imRectRight   右目图像
     * @param[in] timestamp     时间戳
     * @return cv::Mat          世界坐标系到该帧相机坐标系的变换矩阵
     */
    cv::Mat GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp);

    /**
     * @brief 处理RGBD输入的图像
     *
     * @param[in] imRGB         彩色图像
     * @param[in] imD           深度图像
     * @param[in] timestamp     时间戳
     * @return cv::Mat          世界坐标系到该帧相机坐标系的变换矩阵
     */
    cv::Mat GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp);

    /**
     * @brief 处理单目输入图像
     *
     * @param[in] im            图像
     * @param[in] timestamp     时间戳
     * @return cv::Mat          世界坐标系到该帧相机坐标系的变换矩阵
     */
    cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);

    // Load new settings
    // The focal length should be similar or scale prediction will fail when projecting points
    // TODO: Modify MapPoint::PredictScale to take into account focal lenght
    /**
     * @brief
     *
     * @param[in] strSettingPath 配置文件路径
     */
    void ChangeCalibration(const string &strSettingPath);

    // Use this function if you have deactivated local mapping and you only want to localize the camera.
    /**
     * @brief 设置进入仅定位模式
     *
     * @param[in] flag 设置仅仅进行跟踪的标志位
     */
    void InformOnlyTracking(const bool &flag);

    // Tracking states
    enum eTrackingState{
      SYSTEM_NOT_READY=-1,
      NO_IMAGES_YET=0,
      NOT_INITIALIZED=1,
      OK=2,
      LOST=3
      };



  // State of tracking state
  eTrackingState mState;
  // State of last frame
  eTrackingState mLastProcessedState;

  // Input sensor: MONOCULAR, STEREO, RGBD
  int mSensor;

  // Current Frame
  Frame mCurrentFrame;

  cv::Mat mImGray;

  // Initialization Variables (Monocular)
  // Last match
  std::vector<int> mvIniLastMatches;
  std::vector<int> mvIniMatches;
  std::vector<cv::Point2f> mvbPrevMatches;
  std::vector<cv::Point3f> mvIniP3D;
  Frame mInitialFrame;

  // Lists used to recover the full camera trajectory at the end of the execution
  // Basically we store the reference keyframe for each frame and its relative transformation
  // Store whole keyframe pose
  list<cv::Mat> mlRelativeFramePoses;
  // Reference keyframe
  list<KeyFrame*> mlpReferences;
  // Time stamps of frame
  list<double> mlFrameTimes;
  // If lost
  list<bool> mlbLost;

  // True if local mapping is deactivated and we are performing only localization
  bool mbOnlyTracking;

  void Reset();


 protected:

  // Main tracking function. It is independent of the input sensor
  void Track();

  // Map initialization for stereo and RGB-D
  void StereoInitialization();

  // Map initialization for monocular
  void MonocularInitialization();
  void CreateInitialMapMonocular();

  bool TrackWithMotionModel();

  void UpdataLastFrame();

  bool mbVO;


  ///作者自己编写和改良的ORB特征点提取器
  ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
  ///在初始化的时候使用的特征点提取器,其提取到的特征点个数会更多
  ORBextractor* mpIniORBextractor;

  //BoW 词袋模型相关
  ///ORB特征字典
  ORBVocabulary* mpORBVocabulary;
  ///当前系统运行的时候,关键帧所产生的数据库
  KeyFrameDatabase* mpKeyFrameDB;

  // Initalization (only for monocular)
  /// 单目初始器
//  Initializer* mpInitializer;

  //Local Map 局部地图相关
  ///参考关键帧
  KeyFrame* mpReferenceKF;// 当前关键帧就是参考帧
  ///局部关键帧集合
  std::vector<KeyFrame*> mvpLocalKeyFrames;
  ///局部地图点的集合
  std::vector<MapPoint*> mvpLocalMapPoints;

  // System
  ///指向系统实例的指针
//  System* mpSystem;

  //Drawers  可视化查看器相关
  ///查看器对象句柄
//  Viewer* mpViewer;
  ///帧绘制器句柄
  FrameDrawer* mpFrameDrawer;
  ///地图绘制器句柄
  MapDrawer* mpMapDrawer;

  //Map
  ///(全局)地图句柄
  Map* mpMap;

  //Calibration matrix   相机的参数矩阵相关
  ///相机的内参数矩阵
  cv::Mat mK;
  ///相机的去畸变参数
  cv::Mat mDistCoef;
  ///相机的基线长度 * 相机的焦距
  float mbf;

  //New KeyFrame rules (according to fps)
  // 新建关键帧和重定位中用来判断最小最大时间间隔，和帧率有关
  int mMinFrames;
  int mMaxFrames;

  // Threshold close/far points
  // Points seen as close by the stereo/RGBD sensor are considered reliable
  // and inserted from just one frame. Far points requiere a match in two keyframes.
  ///用于区分远点和近点的阈值. 近点认为可信度比较高;远点则要求在两个关键帧中得到匹配
  float mThDepth;

  // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
  ///深度缩放因子,链接深度值和具体深度值的参数.只对RGBD输入有效
  float mDepthMapFactor;

  //Current matches in frame
  ///当前帧中的进行匹配的内点,将会被不同的函数反复使用
  int mnMatchesInliers;

  //Last Frame, KeyFrame and Relocalisation Info
  // 上一关键帧
  KeyFrame* mpLastKeyFrame;
  // 上一帧
  Frame mLastFrame;
  // 上一个关键帧的ID
  unsigned int mnLastKeyFrameId;
  // 上一次重定位的那一帧的ID
  unsigned int mnLastRelocFrameId;

  //Motion Model
  cv::Mat mVelocity;

  //Color order (true RGB, false BGR, ignored if grayscale)
  ///RGB图像的颜色通道顺序
  bool mbRGB;

  ///临时的地图点,用于提高双目和RGBD摄像头的帧间效果,用完之后就扔了
  list<MapPoint*> mlpTemporalPoints;

};
}


#endif //VO_INCLUDE_TRACKING_SELF_H_
