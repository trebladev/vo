//
// Created by xuan on 10/23/22.
//
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
#include "Timing.h"

#include <opencv2/imgproc/imgproc.hpp>

using namespace std;
void stop_falg_detection();

// A flag to indicate whether a key had been pressed.
std::atomic_bool stop_flag(false);

int main(int argc, char **argv) try {

//  if(argc != 3){
//      cerr << endl << "Usage: ./rgbd_realsense path_to_vocabulary path_to_settings" << endl;
//      return EXIT_SUCCESS;
//  }

  std::cout << "Querying Realsense device info..." << std::endl;

  // Create librealsense context for managing devices
  rs2::context ctx;
  auto devs = ctx.query_devices();  // Get device list
  int device_num = devs.size();
  std::cout << "Device number: " << device_num << std::endl; // Device amount

  // Query the info of first device
  rs2::device dev = devs[0];  // If no device conneted, a rs2::error exception will be raised
  // Device serial number (different for each device, can be used for searching device when having mutiple devices)
  std::cout << "Serial number: " << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;

  rs2::config cfg;
  // Default it will config all the devices，you can specify the device index you want to config (query by serial number)
  // Config color stream: 640*480, frame format: BGR, FPS: 30
  cfg.enable_stream(RS2_STREAM_COLOR,640, 480, RS2_FORMAT_BGR8, 60);  // BGR8 correspond to CV_8UC3 in OpenCV
  // Config depth stream: 640*480, frame format: Z16, FPS: 30
  cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 60); // Z16 corresponds to CV_16U in OpenCV

  std::cout << "Config RGB frame format to 8-channal RGB" << std::endl;
  std::cout << "Config RGB and depth FPS to 30" << std::endl;

  rs2::pipeline pipe;
  pipe.start(cfg);

  // Block program until frames arrive
  rs2::frameset data = pipe.wait_for_frames();

  rs2::depth_frame depth = data.get_depth_frame();
  rs2::video_frame color = data.get_color_frame();

  rs2::stream_profile depth_profile = depth.get_profile();
  rs2::stream_profile color_profile = color.get_profile();

  // Get RGB camera intrinsics
  // Note that the change of config will cause the change of intrinsics
  rs2::video_stream_profile cvsprofile(color_profile);
  rs2::video_stream_profile dvsprofile(depth_profile);
  rs2_intrinsics color_intrinsics = cvsprofile.get_intrinsics();
  rs2_intrinsics depth_intrinsics = dvsprofile.get_intrinsics();

  const int color_width = color_intrinsics.width;
  const int color_height = color_intrinsics.height;
  const int depth_width = depth_intrinsics.width;
  const int depth_height = depth_intrinsics.height;

  std::cout << "RGB Frame width: " << color_width << std::endl;
  std::cout << "RGB Frame height: " << color_height << std::endl;
  std::cout << "Depth Frame width: " << depth_width << std::endl;
  std::cout << "Depth Frame height: " << depth_height << std::endl;
  std::cout << "RGB camera intrinsics:" << std::endl;
  std::cout << "fx: " << color_intrinsics.fx << std::endl;
  std::cout << "fy: " << color_intrinsics.fy << std::endl;
  std::cout << "cx: " << color_intrinsics.ppx << std::endl;
  std::cout << "cy: " << color_intrinsics.ppy << std::endl;
  std::cout << "RGB camera distortion coeffs:" << std::endl;
  std::cout << "k1: " << color_intrinsics.coeffs[0] << std::endl;
  std::cout << "k2: " << color_intrinsics.coeffs[1] << std::endl;
  std::cout << "p1: " << color_intrinsics.coeffs[2] << std::endl;
  std::cout << "p2: " << color_intrinsics.coeffs[3] << std::endl;
  std::cout << "k3: " << color_intrinsics.coeffs[4] << std::endl;
  //std::cout << "RGB camera distortion model: " << color_intrinsics.model << std::endl;

  std::cout << "* Please adjust the parameters in config file accordingly *" << std::endl;

//  data = pipe.wait_for_frames();

//  depth = data.get_depth_frame();
//  color = data.get_color_frame();
  std::vector<cv::KeyPoint> allkeypoint;
  cv::Mat alldesc;
  while(1){
    TIMER_START(ORB);
    double t_start = cv::getTickCount();
    data = pipe.wait_for_frames();

    rs2::frame depth = data.get_depth_frame();
    rs2::frame color = data.get_color_frame();

    ORB_SLAM2::ORBextractor* test_extoractor = new ORB_SLAM2::ORBextractor(
        1000,
        1.2,
        8,
        20,
        8
        );
    cv::Mat color_img(cv::Size(depth_width,depth_height), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
    cv::Mat pic_depth(cv::Size(depth_width,depth_height), CV_16U, (void*)depth.get_data(), cv::Mat::AUTO_STEP);

    cv::cvtColor(color_img,color_img,CV_RGBA2GRAY);
    (*test_extoractor)(
        color_img,
        cv::Mat(),
        allkeypoint,
        alldesc
        );

////    std::cout<<"keypoint number"<<allkeypoint.size()<<std::endl;

    cv::drawKeypoints(color_img,allkeypoint,color_img);
//    cv::imshow("depth test",pic_depth);

    double t_end = cv::getTickCount();
    double FPS = cv::getTickFrequency()/(t_end-t_start);
    TIMER_END(ORB);
    cv::Mat fpsPane(35, 155, CV_8UC3);
    fpsPane.setTo(cv::Scalar(153, 119, 76));
    cv::Mat srcRegion = color_img(cv::Rect(8, 8, fpsPane.cols, fpsPane.rows));
    cv::addWeighted(srcRegion, 0.4, fpsPane, 0.6, 0, srcRegion);
    std::stringstream fpsSs;
    fpsSs << "FPS: " << (int)FPS;
    cv::putText(color_img, fpsSs.str(), cv::Point(16, 32),
                cv::FONT_HERSHEY_DUPLEX, 0.8, cv::Scalar(0, 255, 0),2);
//    cout<<"FPS:"<<FPS<<endl;
    cv::imshow("color test",color_img);
    cv::waitKey(1);
    cv::imshow("depth test",pic_depth);
    cv::waitKey(1);
  }

//  // Create SLAM system. It initializes all system threads and gets ready to process frames.
//  ORB_SLAM2::System SLAM(argv[1],argv[2],ORB_SLAM2::System::RGBD,true);
//
//  // Vector for tracking time statistics
//  vector<float> vtimes_track;
//
//  std::thread stop_detect_thread = std::thread(stop_falg_detection);
//
//  std::cout << std::endl << "-------" << std::endl;
//  std::cout << "Start processing realsense stream ..." << std::endl;
//  std::cout << "Use 'e + enter' to end the system" << std::endl;
//
//  while (!stop_flag){
//      data = pipe.wait_for_frames();
//
//      depth = data.get_depth_frame();
//      color = data.get_color_frame();
//      double time_stamp = data.get_timestamp();
//
//      cv::Mat im_D(cv::Size(depth_width, depth_height), CV_16U, (void*)depth.get_data(), cv::Mat::AUTO_STEP);
//      cv::Mat im_RGB(cv::Size(color_width, color_height), CV_8UC3, (void*)color.get_data(), cv::Mat::AUTO_STEP);
//
//      std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
//
//      // Pass the image to the SLAM system
//      SLAM.TrackRGBD(im_RGB,im_D,time_stamp);
//
//      std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
//      double ttrack= std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count();
//      vtimes_track.push_back(ttrack);
//  }
//
//  stop_detect_thread.join();
//
//  // Stop all threads
//  SLAM.Shutdown();
//
//  // Tracking time statistics
//  sort(vtimes_track.begin(),vtimes_track.end());
//  float time_total = 0;
//  for(size_t i = 0; i < vtimes_track.size(); i++){
//      time_total += vtimes_track[i];
//  }
//
//  std::cout << "-------" << std::endl << std::endl;
//  std::cout << "median tracking time: " << vtimes_track[vtimes_track.size() / 2] << std::endl;
//  std::cout << "mean tracking time: " << time_total / vtimes_track.size() << std::endl;
//
//  // Save camera trajectory
//  SLAM.SaveTrajectoryTUM("CameraTrajectory.txt");
//  SLAM.SaveKeyFrameTrajectoryTUM("KeyFrameTrajectory.txt");
//
//  return EXIT_SUCCESS;
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
