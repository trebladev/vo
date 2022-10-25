//
// Created by xuan on 10/25/22.
//

#ifndef VO_INCLUDE_FRAME_H_
#define VO_INCLUDE_FRAME_H_

#include <vector>
#include <ORBextractor.h>

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

  Frame(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timeStamp, ORBextractor* extractorLeft, ORBextractor* extractorRight, )
};
}

#endif //VO_INCLUDE_FRAME_H_
