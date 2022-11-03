//
// Created by xuan on 11/3/22.
//

#ifndef VO_INCLUDE_REALSENSE2_H_
#define VO_INCLUDE_REALSENSE2_H_


#include <librealsense2/rs.hpp>
#include "iostream"

rs2::pipeline start_d435i(const int width, const int height, const int fps, const bool ifdepth);

#endif //VO_INCLUDE_REALSENSE2_H_
