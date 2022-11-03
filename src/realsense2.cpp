//
// Created by xuan on 11/3/22.
//
#include "realsense2.h"

rs2::pipeline start_d435i(const int width, const int height, const int fps, const bool ifdepth) {

  std::cout << "Querying Realsense device info...\n";

  // Create librealsense context for managing devices
  rs2::context ctx;
  auto devs = ctx.query_devices();  // Get device list
  int device_num = devs.size();
  std::cout << "Device number: " << device_num << "\n"; // Device amount

  // Query the info of first device
  rs2::device dev = devs[0];  // If no device conneted, a rs2::error exception will be raised
  // Device serial number (different for each device, can be used for searching device when having mutiple devices)
  std::cout << "Serial number: " << dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << "\n";

  rs2::config cfg;
  // Default it will config all the devices，you can specify the device index you want to config (query by serial number)
  // Config color stream: 640*480, frame format: BGR, FPS: 30
  cfg.enable_stream(RS2_STREAM_COLOR, width, height, RS2_FORMAT_BGR8, fps);  // BGR8 correspond to CV_8UC3 in OpenCV
  // Config depth stream: 640*480, frame format: Z16, FPS: 30
  cfg.enable_stream(RS2_STREAM_DEPTH, width, height, RS2_FORMAT_Z16, fps); // Z16 corresponds to CV_16U in OpenCV

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

  return pipe;

}
