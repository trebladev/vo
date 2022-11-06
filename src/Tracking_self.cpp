//
// Created by xuan on 11/3/22.
//
#include <Tracking_self.h>

namespace ORB_SLAM2 {

ORB_SLAM2::Tracking::Tracking(ORB_SLAM2::ORBVocabulary *pVoc,
                              ORB_SLAM2::FrameDrawer *pFrameDrawer,
                              ORB_SLAM2::MapDrawer *pMapDrawer,
                              ORB_SLAM2::Map *pMap,
                              ORB_SLAM2::KeyFrameDatabase *pKFDB,
                              const rs2::pipeline pipe,
                              const int sensor) :
    mState(NO_IMAGES_YET),
    mSensor(sensor),
    mbOnlyTracking(true),
    mbVO(false),
    mpORBVocabulary(pVoc),
    mpKeyFrameDB(pKFDB),
//                                  mpViewer(NULL),                                     //注意可视化的查看器是可选的，因为ORB-SLAM2最后是被编译成为一个库，所以对方人拿过来用的时候也应该有权力说我不要可视化界面（何况可视化界面也要占用不少的CPU资源）
    mpFrameDrawer(pFrameDrawer),
    mpMapDrawer(pMapDrawer),
    mpMap(pMap),
    mnLastRelocFrameId(0)                               //恢复为0,没有进行这个过程的时候的默认值
{
  // Load camera parameters
  rs2::frameset data = pipe.wait_for_frames();
  rs2::video_frame color = data.get_color_frame();

  // Get camera profile
  rs2::stream_profile color_profile = color.get_profile();
  rs2::video_stream_profile cvsprofile(color_profile);
  rs2_intrinsics color_intrinsics = cvsprofile.get_intrinsics();

  float fx = color_intrinsics.fx;
  float fy = color_intrinsics.fy;
  float cx = color_intrinsics.ppx;
  float cy = color_intrinsics.ppy;

  // Get intrinsics matrix
  cv::Mat K = cv::Mat::eye(3, 3, CV_32F);
  K.at<float>(0, 0) = fx;
  K.at<float>(1, 1) = fy;
  K.at<float>(0, 2) = cx;
  K.at<float>(1, 2) = cy;
  K.copyTo(mK);

  // Get distort parameters
  cv::Mat DistCoef(5, 1, CV_32F);
  DistCoef.at<float>(0) = color_intrinsics.coeffs[0];
  DistCoef.at<float>(1) = color_intrinsics.coeffs[1];
  DistCoef.at<float>(2) = color_intrinsics.coeffs[2];
  DistCoef.at<float>(3) = color_intrinsics.coeffs[3];
  DistCoef.at<float>(4) = color_intrinsics.coeffs[4];
  DistCoef.copyTo(mDistCoef);

  mbf = 46.01;

  float fps = 30;

  mMinFrames = 0;
  mMaxFrames = fps;

  mbRGB = 1;

  // Load ORB parameters
  int nFeatures = 1000;
  float fScaleFactor = 1.2;
  int nLevels = 8;
  int fIniThFAST = 20;
  int fMINThFAST = 8;

  // Construct ORBextractor
  mpORBextractorLeft = new ORBextractor(
      nFeatures,
      fScaleFactor,
      nLevels,
      fIniThFAST,
      fMINThFAST
  );

  mDepthMapFactor = 1000;
  mDepthMapFactor = 1.0f / mDepthMapFactor;

}

bool ORB_SLAM2::Tracking::TrackWithMotionModel() {

  // Init the matcher, min distance < 0.9 * the last, check the rotation
  ORB_SLAM2::ORBmatcher matcher(0.9,true);

  // Update last frame pose according to its reference keyfram
  // Create "visual odometry" points
  //! STEP 1 update last frame's pose. For stereo and RGB-D, will generate temp mappoint
  UpdataLastFrame();

  //! STEP 2 According to prev velocity to get init pose
  mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);

  // Clear current mappoint
  fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

  // Project points seen in previous frame
  // Set search radius
  int th;
  if(mSensor!=1)
    th=15; // Monocular
  else
    th=7;

  //! Step 3 Match with last frame, if cannot match, expend the search radius
  int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th,mSensor==0);

  // If few matches, uses a wider window search
  if(nmatches<20){
    fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
    nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,th*2,mSensor==0);
  }

  if(nmatches<20)
    return false;

  //! STEP 4 Optimize frame pose with all matches
  Optimizer::PoseOptimization(&mCurrentFrame);

  //! STEP 5 Discard outliers
  int nmatchesMap = 0;
  for(int i=0; i<mCurrentFrame.N, ++i){
    if(mCurrentFrame.mvpMapPoints[i]){
      if(mCurrentFrame.mvbOutlier[i]) {
        // If a point is outliers, clear all relationship with it
        MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];

        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
        mCurrentFrame.mvbOutlier[i]=false;
        pMP->mbTrackInView = false;
        pMP->mnLastFrameSeen = mCurrentFrame.mnId;
        nmatches--;
      }
      else if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
        ++nmatchesMap;
    }
  }

  if(mbOnlyTracking)
  {
    mbVO = nmatchesMap<10;
    return nmatches>20;
  }

  //! If match more than 10 point, return tracking successful
  return nmatchesMap>=10;

}
}