/*  
 * 
 * Copyright © 2022 DTU, Christian Andersen jcan@dtu.dk
 * 
 * The MIT License (MIT)  https://mit-license.org/
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 * and associated documentation files (the “Software”), to deal in the Software without restriction, 
 * including without limitation the rights to use, copy, modify, merge, publish, distribute, 
 * sublicense, and/or sell copies of the Software, and to permit persons to whom the Software 
 * is furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies 
 * or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
 * PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE 
 * FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, 
 * ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN 
 * THE SOFTWARE. */


#ifndef UVISION_H
#define UVISION_H

#include <iostream>
#include <sys/time.h>
#include <cstdlib>
#include <sys/socket.h>
#include <sys/types.h>
#include <netdb.h>
#include <thread>
#include <mutex>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>
#include <map>
#include <opencv2/core/types.hpp>
#include <opencv2/core/core.hpp>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <opencv2/aruco.hpp>
#include <opencv2/core/core.hpp>

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>

using namespace std;
// forward declaration



class UVision{
  
public:
  /** setup and request data */
  void setup(int argc, char **argv);
  /** decode an unpacked incoming messages
   * \returns true if the message us used */
  bool decode(char * msg);
  /**
   * Stream to screen - if any screen is available */
  bool processImage(float seconds);
  /**
   * Stream to screen - if any screen is available */
  bool golf_mission();
  /**
   * Stream to screen - if any screen is available */
  bool aruco_mission(float seconds);
  /**
   * Close camera */
  void stop();
  /// camera open flag
  bool camIsOpen = false;
  /// stream images to client (e.g. over ssh)
  bool saveImage = false;
  /// Shutting down
  bool terminate = false;
  /// show image to X-terminal
  bool showImage = false;
  bool streaming = false;
  /**
    * images for manual function using slider */
  cv::Mat dest;
  cv::Mat source;
  int slider1;
  int slider2;
  /**
   * focal length for Sandberg camera in pixels */
  const int focalLength = 1008;
  const float golfBallDiameter = 0.043; // meter
  const float holeDiameter = 0.28; // meter
  float arm_dist = 0.40;
  float aruco_size = 0.035;
  /**
   * camera position in robot coordinates (x (forward), y (left), z (up)) */
  const float camPos[3] = {0.140,0.0, 0.205};       // in meters
  const float camTilt = 23 * M_PI / 180; // in radians
  cv::Mat1f camToRobot;
//   const float st = sin(camTilt);
//   const float ct = cos(camTilt);
//   const cv::Mat1f camToRobot( cos(camTilt),  0.f, st, camPos[0];
//                                          0.f ,  1.f, 0.f , camPos[1];
//                                          -st, 0.f, ct, camPos[2];
//                                          0.f ,  0.f, 0.f , 1.f);



 
  
private:
  /// buffer for captured image
  cv::Mat frame, frame_ud;
  const int MSL = 200;
  char s[200];
  cv::Mat camera_matrix, dist_coeffs;
  /// openCV video capture function
  cv::VideoCapture cap;
  /// thread to keep buffer empty
  thread * listener = NULL; /// thread for listen loop
  static void startloop(UVision * vision); /// To spawn the listen loop as a separate thread, it needs to be static
  void loop(); /// endless loop keeping image buffer empty
  bool getNewestFrame(); /// function to get a fresh frame (newest) from camera  
  bool useFrame = false; /// flag to transfer newest image to 'frame' buffer
  bool gotFrame = false; /// flag for the newest image is available in 'frame'
  int frameSerial = 0;
  mutex dataLock;
  //
  //
  bool findBall = false;
  bool doFindBall();
  cv::Mat debugImg;
  /**
   * Bounding boc for found balls */
  vector<cv::Rect> ballBoundingBox;
  void ballProjectionAndTest();
  //
  bool findAruco = false;
  // 
  /**
   * Calculates color distance between pix and col.
   * distance 0 is total match, else block distance for U and V only
   * \returns d = |du| + |dv| */
  int uvDistance(cv::Vec3b pix, cv::Vec3b col);

  //Golf Variables
  cv::Mat1f pos3drob;
  vector<cv::Vec3i> balls_matrix;

  std::map<float, cv::Mat1f> balls_dict;


  cv::Mat1f holePos;

  cv::Mat1f objPossition;
  bool getBalls(string mode);
  bool loopFrames(float seconds, string obj, string mode);
  void move(cv::Mat1f ballPos, string mode);
  void move_arround();
  void takeBall();
  void releaseBall();
  void prepareArmAruco();
  void takeAruco();
  void releaseAruco();
  cv::Mat1f calc_pos3drob(cv::Vec3i obj, float diameter);
  bool findfHole();
  cv::Mat1f rotate_translate(cv::Mat1f position, float angle);
  void update_robot_pos(float x, float y, float angle);
  void turn_angle(float angle, float vel, float tr);
  void drive(float dist, float vel);
  void goto_aruco_area();
  void execute_instruction(string instruction);

  void go_home();


  int server_fd, new_socket;
  struct sockaddr_in address;


  //For fine tunning of the ball
  int cx_for_ball = 648;
  int cy_for_ball = 454;
  int dx_for_ball = 400;
  int dy_for_ball = 400;

  //position in robot coordinates (x (forward), y (left), z (up)) */
  std::vector<cv::Mat> boundary;
  void addBoundaryPoint(double x, double y);
  bool isWithinBoundary(double x, double y);

  float robot_pos[2];
  float robot_angle;
  cv::Mat1f posToRobot;


  cv::Mat1f robot2orig(float x, float y);
  cv::Mat1f orig2robot(float x, float y);

  const cv::Scalar low_orange = cv::Scalar(0, 45, 100);
  const cv::Scalar up_orange = cv::Scalar(30, 255, 255);

  const cv::Scalar low_blue = cv::Scalar(90,20,200);
  const cv::Scalar up_blue = cv::Scalar(120, 255, 255);

  float distanceThreshold = 0.05;
  int move_arround_dir;

  void stream();
  int socket_val;
  const int port = 8080;

  bool doFindAruco(float seconds, int id);
  cv::Mat1f aruco_location();

  std::map<int, std::vector<cv::Vec3i>> markerSamples;
  // struct markerSamples {
  //   int id;
  //   std::vector<cv::Vec3i> samples;
  // };


  float minCornerX; 
  int leftMarkerId; 
  int maxDist_aruco = 2;
  cv::Vec3i leftArucoFramePos;
  cv::Mat1f aruco_pos;
  


  
};

/**
 * Make this bridge connection visible to the rest of the software */
extern UVision vision;

#endif