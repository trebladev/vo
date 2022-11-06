//
// Created by xuan on 11/3/22.
//
// This file is test how to track frame 2 frame

#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <atomic>
#include <thread>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include "include/ORBextractor.h"
#include <librealsense2/rs.hpp>
#include "opencv2/features2d/features2d.hpp"
#include "ORBVocabulary.h"
#include "Timing.h"
#include "Frame.h"
#include "KeyFrame.h"
#include "FrameDrawer.h"
#include "Map.h"
#include "realsense2.h"
#include <opencv2/imgproc/imgproc.hpp>

#define IMAGE_WIDTH 640
#define IMAGE_HEIGHT 480
#define FPS     30

using namespace std;
void stop_falg_detection();

// A flag to indicate whether a key had been pressed.
std::atomic_bool stop_flag(false);

int main(int argc, char **argv) try {

  std::string strSettingsFile = "./test.yaml";

  rs2::pipeline pipe = start_d435i(IMAGE_WIDTH,IMAGE_HEIGHT,FPS,true);
  rs2::frameset data;

  // Load vocabulary
  ORB_SLAM2::ORBVocabulary* Vocabulary = new ORB_SLAM2::ORBVocabulary();
  bool bvocload = Vocabulary->loadFromTextFile("./Vocabulary/ORBvoc.txt");
  if(!bvocload){
    cerr<< "Wrong path to vocabulary \n";
    exit(-1);
  }
  cout<< "Load vocabulary successful\n";

  // The code is used in system.h
  // Create KeyFrame Database
  ORB_SLAM2::KeyFrameDatabase* KeyFrameDatabase = new ORB_SLAM2::KeyFrameDatabase(*Vocabulary);

  // Create the Map
  ORB_SLAM2::Map* Map = new ORB_SLAM2::Map();

  //Create Drawers. These are used by the Viewer
  ORB_SLAM2::FrameDrawer* mpFrameDrawer = new ORB_SLAM2::FrameDrawer(Map);
  ORB_SLAM2::MapDrawer* MapDrawer = new ORB_SLAM2::MapDrawer(Map, strSettingsFile);

  // Init Tracking
  ORB_SLAM2::Tracking* Tracking = new ORB_SLAM2::Tracking(Vocabulary,
                                                          mpFrameDrawer,
                                                          MapDrawer,
                                                          Map,
                                                          KeyFrameDatabase,
                                                          pipe,
                                                          2);

  std::cout<< "Init Tracking successful !\n";

  while(1){
    data = pipe.wait_for_frames();

    rs2::frame depth = data.get_depth_frame();
    rs2::frame color = data.get_color_frame();


    cv::Mat color_img(cv::Size(IMAGE_WIDTH,IMAGE_HEIGHT), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);

    cv::imshow("color test",color_img);
    cv::waitKey(1);
  }

  return 0;


}catch(const rs2::error &e){
  // Capture device exception
  std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
  return EXIT_FAILURE;
}catch(const std::exception &e){
  std::cerr<<"Other error : " << e.what() << std::endl;
  return EXIT_FAILURE;
}

void stop_falg_detection(){
  char c;
  while (!stop_flag) {
    c = std::getchar();
    if(c == 'e'){
      stop_flag = true;;
    }
  }
}