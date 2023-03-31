/*  
 * 
 * Copyright © 2022 DTU, 
 * Author:
 * Christian Andersen jcan@dtu.dk
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

#include <string>
#include <string.h>
#include "ubridge.h"
#include "uvision.h"
#include "utime.h"
#include "uplay.h"
#include "uevent.h"
#include <opencv2/imgproc.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <sys/socket.h>
#include <netinet/in.h>
#include <unistd.h>
#include <map>
using namespace std;
using namespace cv;

// create the vision object
UVision vision;

#define PORT 8080


// class implementation:
void UVision::setup(int argc, char **argv)
{ /// Setup camera and start streaming
  // open the default camera using default API
  // and select any API backend
  int dev = 0;
  //
  // decode any debug parameters
  // command line parameters
  for (int i = 1; i < argc; i++)
  { // check for command line parameters
    // for process debug
    if (strcmp(argv[i], "save") == 0)
      saveImage = true;
    if (strcmp(argv[i], "ball") == 0)
      findBall = true;
    if (strcmp(argv[i], "aruco") == 0)
      findAruco = true;
    if (strcmp(argv[i], "show") == 0)
      showImage = true;
    if (strcmp(argv[i], "streaming") == 0)
      streaming = true;
    if (strncmp(argv[i], "video", 5) == 0)
    {
      const char * p1 = argv[i];
      p1 += 5;
      dev = strtol(p1, nullptr, 10);
    }
  }
  // prepare to open camera
  int deviceID = dev;             // 0 = open default camera
  int apiID = cv::CAP_V4L2;  //cv::CAP_ANY;  // 0 = autodetect default API
  // open selected camera using selected API
  //   cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280);
  cap.open(deviceID, apiID);
  // check if we succeeded
  camIsOpen = cap.isOpened();
  if (not camIsOpen)
  {
    cerr << "ERROR! Unable to open camera\n";
  }
  else
  {
    uint32_t fourcc = cv::VideoWriter::fourcc('M','J','P','G'); 
    cap.set(cv::CAP_PROP_FOURCC, fourcc);
    // possible resolutions in JPEG coding
    // (rows x columns) 320x640, 720x1280
    cap.set(cv::CAP_PROP_FRAME_HEIGHT, 720); // 320
    cap.set(cv::CAP_PROP_FRAME_WIDTH, 1280); // 640
    //cap.set(cv::CAP_PROP_FRAME_HEIGHT, 320); // 320
    //cap.set(cv::CAP_PROP_FRAME_WIDTH, 640); // 640
    cap.set(cv::CAP_PROP_FPS, 25); // 640
    union FourChar
    {
      uint32_t cc4;
      char ccc[4];
    } fmt;
    fmt.cc4 = cap.get(cv::CAP_PROP_FOURCC);
    printf("# Video device %d: width=%g, height=%g, format=%c%c%c%c, FPS=%g\n", 
           dev, 
           cap.get(cv::CAP_PROP_FRAME_WIDTH), 
           cap.get(cv::CAP_PROP_FRAME_HEIGHT), 
           fmt.ccc[0], fmt.ccc[1], fmt.ccc[2], fmt.ccc[3], 
           cap.get(cv::CAP_PROP_FPS));
    //
    // start thread to keep buffer empty
    printf("# Vision::setup: Starting image capture loop\n");
    listener = new thread(startloop, this);
  }
  // generate projection matrix
  float st = sin(camTilt);
  float ct = cos(camTilt);
  camToRobot = (cv::Mat1f(4,4) << ct,  0.f, st, camPos[0],
                                  0.f ,  1.f, 0.f , camPos[1],
                                  -st, 0.f, ct, camPos[2],
                                  0.f ,  0.f, 0.f , 1.f);
  //
  //READ CALIBRATION FILE
  cv::FileStorage fs("../calibration.yaml", cv::FileStorage::READ);


  if (fs.isOpened())
  {
      fs["camera_matrix"] >> camera_matrix;
      fs["dist_coeff"] >> dist_coeffs;
      fs.release();
  }
  else
  {
      std::cerr << "Failed to open YAML file." << std::endl;
  }

  std::cout << "Camera matrix: " << std::endl << camera_matrix << std::endl;
  std::cout << "Distortion coefficients: " << std::endl << dist_coeffs << std::endl;

  if (streaming){

    std::cout << "CONNECTION SETUP!!!"  << std::endl;

    int server_fd, opt = 1;
    struct sockaddr_in address;
    int addrlen = sizeof(address);

    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0) {
        std::cerr << "socket failed"<< std::endl;
        exit(EXIT_FAILURE);
    }

    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
        std::cerr << "setsockopt"<< std::endl;
        exit(EXIT_FAILURE);
    }

    address.sin_family = AF_INET;
    address.sin_addr.s_addr = htonl(INADDR_ANY);
    address.sin_port = htons(PORT);

    std::cout << "CONNECTION33!!!"  << std::endl;
    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address)) < 0) {
        std::cerr << "bind failed"<< std::endl;
        exit(EXIT_FAILURE);
    }

    std::cout << "CONNECTION44!!!"  << std::endl;
    if (listen(server_fd, 3) < 0) {
        std::cerr << "listen"<< std::endl;
        exit(EXIT_FAILURE);
    }

    std::cout << "CONNECTION55!!!"  << std::endl;

    if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen)) < 0) {
        std::cerr << "accept"<< std::endl;
        exit(EXIT_FAILURE);
    }
    std::cout << "CONNECTION66!!!"  << std::endl;

    close(server_fd);

    std::cout << "CONNECTION!!!"  << std::endl;


  }

}

void UVision::stop()
{
  if (camIsOpen)
  {
    camIsOpen = false;
    // allow last frame to finish
    usleep(300000);
    // close
    cap.release();
  }
  close(new_socket);
}

bool UVision::decode(char* msg)
{ // no incoming vision data from bridge
  return false;
}

void UVision::startloop(UVision* vision)
{ // start camera loop (thread)
  vision->loop();
}

void UVision::loop()
{
  while (camIsOpen and not terminate)
  { // keep framebuffer empty
    if (useFrame)
    { // grab and decode next image
      cap.read(frame);
      // mark as available
      gotFrame = not frame.empty();
      useFrame = not gotFrame;
      if (gotFrame){
        undistort(frame, frame_ud, camera_matrix, dist_coeffs);

      }

    }
    else
      // just grab the image - mark it as used
      cap.grab();
    frameSerial++;
  }
}

bool UVision::getNewestFrame()
{ // request new frame
  gotFrame = false;
  useFrame=true;
  // allow timeout, 1 second from now
  UTime t;
  t.now();
  while (not gotFrame and t.getTimePassed() < 1.0)
  { // wait for frame (or timeout of 1 second)
    usleep(3000);
  }
  if (not gotFrame)
    printf("# failed to get an image frame\n");
  return gotFrame;
}


cv::Mat1f UVision::calc_pos3drob(cv::Vec3i obj, float diameter){
  int radio = obj[2];
  float dist = diameter * focalLength / float(radio*2);
  // the position in x (right) and y (down)

  float bbCenter[2] = {obj[0], obj[1]};
  float frameCenter[2] = {frame.cols/2.0f, frame.rows/2.0f};

  // distance right of image center line - in meters
  float x = (bbCenter[0] - frameCenter[0])/focalLength * dist;
  // distance below image center line - in meters
  float y = (bbCenter[1] - frameCenter[1])/focalLength * dist;
  // make a vector of ball center with (x=forward, y=left, z=up)
  cv::Vec4f pos3dcam(dist, -x, -y, 1.0f);
  // printf("# ball %d position in cam   coordinates (x,y,z)=(%.2f, %.2f, %.2f)\n", 
  //       pos3dcam[0], pos3dcam[1], pos3dcam[2]);
  // print used matrices and vector
  // cout << "camToRobot: " << camToRobot << "\n";
  // cout << "# pos3dcam  : " << pos3dcam << "\n";
  cv::Mat1f pos3drobot = camToRobot * pos3dcam;
  
  return pos3drobot;       
}



bool UVision::getBalls(string mode="GROSS"){
  bool ball= false;
  Vec3i f_circle(0,0,0);
  Mat frame_HSV, frame_gray, frame_threshold, frame_ed, frame_prep, mask;
  int min_radio = 40;

  if (mode == "FINE"){

    cv::Rect rect(cx_for_ball-dx_for_ball/2, cy_for_ball-dy_for_ball/2, dx_for_ball, dy_for_ball);

    // Step 2: Create a binary mask image
    cv::Mat mask(frame_ud.rows, frame_ud.cols, CV_8UC1, cv::Scalar(0));

    // Step 3: Set all pixels outside of the rectangle to 0 and all pixels inside the rectangle to 1
    cv::rectangle(mask, rect, cv::Scalar(255), -1); // -1 means fill the rectangle

    // Step 4: Apply the mask to the input image
    frame_ud.copyTo(frame_prep, mask);
    min_radio = 70;
  }
  else{
    cv::cvtColor(frame_ud, frame_HSV, COLOR_BGR2HSV);
    cv::blur(frame_HSV, frame_HSV, cv::Size(7, 7));
    cv::inRange(frame_HSV, low_orange, up_orange ,frame_threshold);
    cv::erode(frame_threshold, frame_ed, cv::Mat(), cv::Point(-1,-1), 5);
    cv::dilate(frame_ed, frame_ed, cv::Mat(), cv::Point(-1,-1), 40);
    frame_ud.copyTo(frame_prep, frame_ed);
  }
  if (showImage)
  {
    imshow("THR", frame_prep);
  }


  cv::cvtColor(frame_prep, frame_gray, COLOR_BGR2GRAY);
  cv::GaussianBlur(frame_gray, frame_gray, Size(9, 9), 0);

  vector<Vec3f> circles;
  // cv::HoughCircles(frame_gray, circles, HOUGH_GRADIENT, 1,
  //             frame_gray.rows/4,  // change this value to detect circles with different distances to each other
  //             200, 5, 40, 100 // change the last two parameters
  //         // (min_radius & max_radius) to detect larger circles
  // );
  cv::HoughCircles(frame_gray, circles, HOUGH_GRADIENT, 1,
              frame_gray.rows/4,  // change this value to detect circles with different distances to each other
              100, 25, min_radio, 100 // change the last two parameters
          // (min_radius & max_radius) to detect larger circles
  );
  for( size_t i = 0; i < circles.size(); i++ )
  {
      Vec3i c = circles[i];
      Point center = Point(c[0], c[1]);
      // circle center


      if (0<(c[0]-c[2]/4) && (c[0]+c[2]/4)<frame.cols && 0<(c[1]-c[2]/4) && (c[1]+c[2]/4)<frame.rows){
        
        cv::cvtColor(frame_ud, frame_HSV, COLOR_BGR2HSV);
        cv::Mat roi = frame_HSV(cv::Range(c[1]-c[2]/8, c[1]+c[2]/8), cv::Range(c[0]-c[2]/8, c[0]+c[2]/8));
        cv::Mat1b mask(roi.rows, roi.cols);
        cv::Scalar mean = cv::mean(roi, mask);
        int hue = round(mean[0]);
        int sat = round(mean[1]);
        int val = round(mean[2]);
        cout << "CIRCLE FOUND: " << center << " radio: " << c[2] << " hue: " << hue << " sat: " << sat << " val: " << val <<"\n";

        if (hue< 30 and val>160){
        //if (hue< 30){
          cout << "BALL FOUND: " << center;
          cout << " radio: " << c[2] << " hue: " << hue << "\n";
          cv::circle( frame_ud, center, 1, Scalar(0,100,100), 3, LINE_AA);
          // circle outline
          int radius = c[2];
          cv::circle( frame_ud, center, radius, Scalar(255,0,255), 3, LINE_AA);
          std::string text = "BALL FOUND: (" + std::to_string(center.x) + ", " + std::to_string(center.y) + ")" + " radio: " + std::to_string(c[2]) + "hsv" + std::to_string(hue) + "," + std::to_string(sat)+ "," + std::to_string(val); 
          cv::putText(frame_ud, text, center, cv::FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(0,0,0), 2, false);
          
          f_circle[0] = c[0];
          f_circle[1] = c[1];
          f_circle[2] = radius;

          balls_dict[radius] = calc_pos3drob(f_circle, golfBallDiameter);
          ball = true;


        }
      }
  }
  return ball;
}

bool UVision::findfHole(){
  cout << "FIND HOLE\n";
  bool hole= false;
  Vec3i f_hole(0,0,0);
  Mat frame_HSV, frame_gray, frame_threshold, frame_ed;
  cv::cvtColor(frame_ud, frame_HSV, COLOR_BGR2HSV);
  cv::blur(frame_HSV, frame_HSV, cv::Size(5, 5));
  // Lower values: 210.678 81.12039999999999 74.228
  // Upper values: 230.678 181.1204 174.228
  
  cv::inRange(frame_HSV, low_blue, up_blue,frame_threshold);
  //cv::inRange(frame_YUV, Scalar(210,80,74), Scalar(230, 181, 174),frame_threshold);
  //cv::inRange(frame_YUV, Scalar(165,130,120), Scalar(200, 140, 135),frame_threshold);

  cv::erode(frame_threshold, frame_ed, cv::Mat(), cv::Point(-1,-1), 2);
  cv::dilate(frame_ed, frame_ed, cv::Mat(), cv::Point(-1,-1), 2);

  // //
  // // find contours for further validation
  // vector<vector<cv::Point> > contours;
  // vector<cv::Vec4i> hierarchy; // not used, but needed
  // cv::findContours(frame_ed, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE );

  // cv::Moments moments = cv::moments(contours[0]);

  // // Get the centroid of the largest contour
  // cv::Point2f centroid(moments.m10 / moments.m00, moments.m01 / moments.m00);

  // // Get the area of the largest contour
  // double area = cv::contourArea(contours[0]);

  // // Print the centroid and area
  // std::cout << "Centroid: (" << centroid.x << ", " << centroid.y << ")" << std::endl;
  // std::cout << "Area: " << area << std::endl;

  // if (area>40){
  //   // Draw the largest contour and its centroid on a new image
  //   cv::Mat result = cv::Mat::zeros(frame_ed.size(), CV_8UC3);
  //   cv::drawContours(frame, contours, 0, cv::Scalar(0, 255, 0), 2);
  //   cv::circle(frame, centroid, 5, cv::Scalar(0, 0, 255), -1);

  //   f_hole[0] = moments.m10 / moments.m00;
  //   f_hole[1] = moments.m01 / moments.m00;
  //   f_hole[2] = sqrt(area)/2;

  //   holePos = calc_pos3drob(f_hole, holeDiameter);
  //   hole = true;
    
  // }


  cv::Mat labels, stats, centroids;
  int num_labels = cv::connectedComponentsWithStats(frame_ed, labels, stats, centroids);
  cv::Mat result = cv::Mat::zeros(frame_ed.size(), CV_8UC3);
  int max_area = -1;
  int max_label = -1;
  for (int i = 1; i < num_labels; i++) {
      int area = stats.at<int>(i, cv::CC_STAT_AREA);
      if (area > max_area) {
          max_area = area;
          max_label = i;
      }
  }


  // Print the centroid and area
  std::cout << "Centroid: (" << centroids.at<double>(max_label, 0) << ", " << centroids.at<double>(max_label, 1) << ")" << std::endl;
  std::cout << "Area: " << max_area << std::endl;

  if (max_area>40){


    f_hole[0] = centroids.at<double>(max_label, 0);
    f_hole[1] = centroids.at<double>(max_label, 1);
    f_hole[2] = stats.at<int>(max_label, cv::CC_STAT_WIDTH)/2;

    holePos = calc_pos3drob(f_hole, holeDiameter);
    hole = true;
    
  }

  if (showImage)
  {
    imshow("HOLE", frame_ed);
    // Draw bounding box around the component with the maximum area
    if (max_label != -1) {
        cv::Rect bbox(stats.at<int>(max_label, cv::CC_STAT_LEFT),
                      stats.at<int>(max_label, cv::CC_STAT_TOP),
                      stats.at<int>(max_label, cv::CC_STAT_WIDTH),
                      stats.at<int>(max_label, cv::CC_STAT_HEIGHT));
        cv::rectangle(frame_ud, bbox, cv::Scalar(0, 0, 255), 2);
    }

    imshow("HOLE_HSV", frame_HSV);
  }


  return hole;
}



bool UVision::loopFrames(float seconds, string obj, string mode)
{
  cout << "# LOOP FRAMES FOR" << obj << "\n";

  vector<cv::Mat1f> samples;
  cv::Mat1f accumulated = cv::Mat1f::zeros(1, 3);  // Accumulated sum
  int numSamples = 0;


  UTime t;
  t.now();

  while (frameSerial < 30){
    cout << "# wait for the first frames\n";
    getNewestFrame(); 

  }

  t.now();


  //while (t.getTimePassed() < seconds and camIsOpen and not terminate and n<5) {
  while (t.getTimePassed() < seconds and camIsOpen and not terminate and numSamples < 20) {
    //cap >> image;

    getNewestFrame(); 

    bool found = false;
    if (gotFrame){
      if (obj == "BALL"){
        found = getBalls(mode);
        if (found==true){

          float max_radius = std::numeric_limits<float>::min();
          for (const auto& it : balls_dict)
          {
              if (it.first > max_radius)
              {
                  max_radius = it.first;
                  pos3drob = it.second;
              }
          }

          balls_dict.erase(max_radius);
        }
      }


      if (obj == "HOLE"){
        found = findfHole();
        if (found==true){
          pos3drob = holePos;
        }
      }



      if (found==true){
        
        printf("# Object position in robot coordinates (x,y,z)=(%.2f, %.2f, %.2f)\n", 
              pos3drob.at<float>(0), pos3drob.at<float>(1), pos3drob.at<float>(2));
        
        std::vector<float> sample = pos3drob;

        std::vector<float> sample3 = {sample[0], sample[1], sample[2]};

        cv::Mat1f sampleMat = cv::Mat1f(sample3, true).t();
        //sampleMat.resize(3);
        cout << "sample: " << sampleMat;
        samples.push_back(sampleMat);
        accumulated += sampleMat;
        numSamples += 1;

      }
      //cout << "# pos3dcam  : " << pos3dcam << "\n";

      if (showImage)
      {
        imshow("IMG_FRAMES", frame_ud);
        //imshow("RAW", frame);

      }
      
      waitKey(25);
    }
  }
 // Calculate mean
  cv::Mat1f mean = accumulated / static_cast<float>(numSamples);

  // Filter outliers
  std::vector<cv::Mat1f> filteredSamples;
  for (auto& sample : samples) {
      float distance = cv::norm(sample - mean);
      //cout << "distance : " << distance << "\n";
      if (distance <= distanceThreshold) {
          filteredSamples.push_back(sample);
      }
  }

  // Recalculate mean using only filtered samples
  cv::Mat1f filteredAccumulated = cv::Mat1f::zeros(1, 3);
  for (auto& sample : filteredSamples) {
      filteredAccumulated += sample;
  }
  objPossition = filteredAccumulated / static_cast<float>(filteredSamples.size());

  // Print mean
  cout << "Mean: " << objPossition << std::endl;
  cout << "Samples " << samples.size() << std::endl;

  if (samples.size()>5){
    
    return true;

  }else{

    return false;

  }



}

void UVision::addBoundaryPoint(double x, double y) {
    boundary.push_back((cv::Mat_<double>(2, 1) << x, y));
}

bool UVision::isWithinBoundary(double x, double y) {
    // check if the point is inside the boundary using the "ray casting" algorithm
    // https://en.wikipedia.org/wiki/Point_in_polygon#Ray_casting_algorithm
    int intersections = 0;
    for (int i = 0; i < boundary.size(); i++) {
        double x1 = boundary[i].at<double>(0, 0);
        double y1 = boundary[i].at<double>(0, 1);
        double x2 = boundary[(i+1)].at<double>(0, 0);
        double y2 = boundary[(i+1)].at<double>(0, 1);

        std::cout << "boundary i: " << boundary[i] << std::endl;
        std::cout << "boundary i+1: " << boundary[i]+1 << std::endl;
        if (((y1 > y) != (y2 > y)) && (x < (x2 - x1) * (y - y1) / (y2 - y1) + x1)) {
            intersections++;
        }
    }
    return (intersections % 2 != 0);
}


void UVision::update_robot_pos(float x, float y, float angle){
  

  cv::Vec3f pos_dest(x, y, 1.0f);


  std::cout << "PARAMS TO UPDATE : " << x << "," << y << " angle " << angle << std::endl;

  float angle_radians = robot_angle * M_PI / 180;
  float st = sin(angle_radians);
  float ct = cos(angle_radians);
  posToRobot = (cv::Mat1f(3,3) << ct, -st , robot_pos[0],
                                  st,  ct , robot_pos[1],
                                  0.f, 0.f, 1.f);

  std::cout << "Matrix: " << posToRobot << std::endl;
  cv::Mat1f posrobot = posToRobot * pos_dest;



  std::cout << "OLD ROBOT POS: " << robot_pos[0] << "," << robot_pos[1] << " angle " << robot_angle << std::endl;

  robot_pos[0] = posrobot.at<float>(0, 0);
  robot_pos[1] = posrobot.at<float>(0, 1);
  robot_angle += angle;

  std::cout << "NEW ROBOT POS: " << robot_pos[0] << "," << robot_pos[1] << " angle " << robot_angle << std::endl;

}

cv::Mat1f UVision::robot2orig(float x, float y){
  float angle_radians = robot_angle * M_PI / 180;
  float st = sin(angle_radians);
  float ct = cos(angle_radians);
  posToRobot = (cv::Mat1f(3,3) << ct, -st , robot_pos[0],
                                  st,  ct , robot_pos[1],
                                  0.f, 0.f, 1.f);

  cv::Vec3f pos_dest(x, y, 1.0f);
  cv::Mat1f posrobot = posToRobot * pos_dest;
  return posrobot;
}

cv::Mat1f UVision::orig2robot(float x, float y){ 
  float angle_radians = robot_angle * M_PI / 180;
  float st = sin(angle_radians);
  float ct = cos(angle_radians);
  posToRobot = (cv::Mat1f(3,3) << ct, -st , robot_pos[0],
                                  st,  ct , robot_pos[1],
                                  0.f, 0.f, 1.f); 
  cv::Vec3f pos_dest(x, y, 1.0f);
  cv::Mat1f posrobot = posToRobot.inv() * pos_dest;
  return posrobot;
}


void UVision::move(cv::Mat1f position, string mode=""){
  float arm_dist = 0.38;
  const int MSL = 200;
  char s[MSL];
  double radians = atan(position.at<float>(0, 1) / position.at<float>(0, 0));
  double degrees = radians * (180.0 / M_PI);
  float dist = sqrt(pow(position.at<float>(0, 1),2) + pow(position.at<float>(0, 0),2)) - arm_dist;


  if (mode=="backward"){
    dist = -1*dist;
    degrees = 0;    
  }

  float x = dist*cos(radians);
  float y = dist*sin(radians);

  if (true){

    update_robot_pos(x, y, degrees);
    //float x_p = 

    std::cout << "Angle " << degrees << " degrees " << " Distance: "<< dist << std::endl;
    sound.say(". Object Found.", 0.3);
    // remove old mission
    bridge.tx("regbot mclear\n");
    // clear events received from last mission
    event.clearEvents();
    //usleep(2000);

    bridge.tx("regbot madd vel=0.0, log=3.0: time=0.02\n");

    snprintf(s,MSL,"regbot madd vel=%.2f,tr=0.2:turn=%.1f\n", 0.2, degrees);
    bridge.tx(s);
    std::cout << s << std::endl;

    bridge.tx("regbot madd vel=0.0: time=0.08\n");

    snprintf(s,MSL,"regbot madd vel=0.2:dist=%.2f\n", dist);
    std::cout << s << std::endl;
    bridge.tx(s);

    bridge.tx("regbot madd vel=0.0: time=0.01\n");

    bridge.tx("regbot start\n");
    event.waitForEvent(0);  
    cout << "Move DONE...\n";

  }else{

    cout << "IT IS LOST--> Out of the track...\n";

  }

}



void UVision::go_home(){

  // cv::Point2f original_point{0.0, 0.0};
  // cv::Point2f point{robot_pos[0], robot_pos[1]};
  float x = -robot_pos[0];
  float y = -robot_pos[1];
  float dist = sqrt(pow(x,2) + pow(y,2));


  robot_angle = 180 - robot_angle;
  std::cout << "Position of origin: (" << x << ", " << y << ")" << std::endl;
  std::cout << "robot_angle!!: " << robot_angle << std::endl;
  
  //update_robot_pos(x, y, robot_angle);

  const int MSL = 200;
  char s[MSL];

  // cv::Mat1f new_pos = rotate_translate(robot_pos, robot_angle);
  // move(new_pos);
  std::cout << "Angle " << robot_angle << " degrees " << " Distance: "<< dist << std::endl;
  sound.say(". Object Found.", 0.3);
  // remove old mission
  bridge.tx("regbot mclear\n");
  // clear events received from last mission
  event.clearEvents();
  //usleep(2000);

  bridge.tx("regbot madd vel=0.0, log=3.0: time=0.02\n");

  snprintf(s,MSL,"regbot madd vel=%.2f,tr=0.2:turn=%.1f\n", 0.2, robot_angle);
  bridge.tx(s);
  std::cout << s << std::endl;

  bridge.tx("regbot madd vel=0.0: time=0.08\n");

  snprintf(s,MSL,"regbot madd vel=0.2:dist=%.2f\n", dist);
  std::cout << s << std::endl;
  bridge.tx(s);

  bridge.tx("regbot madd vel=0.0: time=0.01\n");

  bridge.tx("regbot start\n");
  event.waitForEvent(0);  

  cout << "HOME...\n";
}


void UVision::takeBall(){
  //servo=1,pservo=-600, vservo=120:time=8

  bridge.tx("regbot mclear\n");
  // clear events received from last mission
  event.clearEvents();
  //usleep(2000);
 
  bridge.tx("regbot madd servo=2,pservo=100:time=2\n");
  bridge.tx("regbot madd servo=1,pservo=-600,vservo=120:time=8\n");
  bridge.tx("regbot madd servo=2,pservo=-600:time=2\n");
  bridge.tx("regbot madd servo=1,pservo=200,vservo=120:time=8\n");

  bridge.tx("regbot start\n");
  cout << "Taking a ball...\n";
  event.waitForEvent(0); 

}

void UVision::releaseBall(){
  //servo=1,pservo=-600, vservo=120:time=8

  bridge.tx("regbot mclear\n");
  // clear events received from last mission
  event.clearEvents();
  //usleep(2000);
 
  bridge.tx("regbot madd servo=1,pservo=-600,vservo=120:time=8\n");
  bridge.tx("regbot madd servo=2,pservo=100:time=2\n");
  bridge.tx("regbot madd servo=1,pservo=200,vservo=120:time=8\n");

  bridge.tx("regbot start\n");
  cout << "Taking a ball...\n";
  event.waitForEvent(0); 

}

void UVision::move_arround(){
  cv::Vec3f vec3f(0.2f, 0.2f, 0.0f);
  cv::Mat1f position = cv::Mat1f(vec3f);
  move(position);
}

bool UVision::golf_mission(){
  int n = 1;
  UTime t;
  t.now();
  //float val = 0.0;
  robot_pos[0] = 0.0;
  robot_pos[1] = 0.0;
  robot_angle = 0.0;
  update_robot_pos(0, 0, 0);

  addBoundaryPoint(0.0, 1.0);
  addBoundaryPoint(5.0, 1.0);
  addBoundaryPoint(0.0, -3.0);
  addBoundaryPoint(5.0, -3.0);

  while(n<3 and t.getTimePassed() < 40){
  //while(n<6){
    bool res = false;
    res = loopFrames(20, "BALL", "GROSS");
    if (res == true){
      std::cout << "# BALL FOUND "<< n <<" Pos: " << objPossition << "\n";
      move(objPossition);
      //usleep(2000);
      
      res = loopFrames(5, "BALL", "FINE");
      //res = false;
      if (res == true){
        std::cout << "# BALL FINE FOUND "<< n <<" Pos: " << objPossition << "\n";
        move(objPossition);
        takeBall();
        
        res = loopFrames(20, "HOLE", "GROSS");
        if (res == true){
          std::cout << "# HOLE FOUND "<< n <<" Pos: " << objPossition << "\n";
          move(objPossition);
          releaseBall();
          move(objPossition,"backward");
          n +=1;
        }
      }
      
    }
    else {
      //MOVE ARROUND
      move_arround();


    }


  }
  go_home();


  return true;
}


bool UVision::doFindAruco(float seconds)
{ // image is in 'frame'
  printf("# GET ARUCO\n");

  UTime t;
  t.now();

  while (t.getTimePassed()< 0.8){
    cout << "# wait for the first frames\n";
    getNewestFrame(); 

  }

  t.now();
  Ptr<aruco::Dictionary> dictionary = cv::makePtr<aruco::Dictionary>(aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100));



  //while (t.getTimePassed() < seconds and camIsOpen and not terminate and n<5) {
  while (t.getTimePassed() < seconds and camIsOpen and not terminate) {
    //cap >> image;

    getNewestFrame(); 

    if (gotFrame){
      
      Mat markerImage;
      // Load the predefined dictionary
      //Ptr<cv::aruco::Dictionary> dictionary = aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100);
      //cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

          
      // Detect the markers in the image
      std::vector<int> markerIds;
      std::vector<std::vector<Point2f>> markerCorners;
      aruco::detectMarkers(frame, dictionary, markerCorners, markerIds);
      
      // Draw the marker outlines on the image
      aruco::drawDetectedMarkers(frame, markerCorners, markerIds);

      if (showImage)
      {
        imshow("ARUCO", frame);
      }

      waitKey(25);
    }
  }
  return false;
}




//cv::Vec3b yuvOrange = cv::Vec3b(128,88,187);
// cv::cvtColor(frame, frame_HSV, COLOR_BGR2HSV);
// cv::imwrite("hsv_picture.png", frame_HSV);
// cv::blur(frame_HSV, frame_HSV, cv::Size(1, 1));
// cv::inRange(frame_HSV, Scalar(7, 50, 50), Scalar(15, 200, 200),frame_threshold);
// cv::erode(frame_threshold, frame_ed, cv::Mat(), cv::Point(-1,-1), 2);
// cv::dilate(frame_ed, frame_ed, cv::Mat(), cv::Point(-1,-1), 2);

// Moments m = moments(frame_ed, false);
// Point com(m.m10 / m.m00, m.m01 / m.m00);

// Scalar color = cv::Scalar(0, 0, 255);
// cv::drawMarker(frame, com, color, cv::MARKER_CROSS, 50, 5);

// cv::imshow("Display window", frame);
// cv::imshow("Display Threshold", frame_threshold);

// cv::imshow("Display ERODED-DILATED", frame_ed);
//doFindBall();
