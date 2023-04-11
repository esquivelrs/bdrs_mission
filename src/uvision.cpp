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
#include <arpa/inet.h>
#include <sys/ioctl.h>
#include <net/if.h>


using namespace std;
using namespace cv;

// create the vision object
UVision vision;





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
    int localSocket, remoteSocket, reuseaddr = 1;
    struct sockaddr_in localAddr, remoteAddr;
    int addrLen = sizeof(struct sockaddr_in);



    localSocket = socket(AF_INET, SOCK_STREAM, 0);
    if (localSocket == -1)
    {
        perror("socket() call failed!!");
        exit(1);
    }

    if (setsockopt(localSocket, SOL_SOCKET, SO_REUSEADDR, &reuseaddr, sizeof(int)) == -1)
    {
        perror("setsockopt");
        exit(1);
    }

    localAddr.sin_family = AF_INET;
    localAddr.sin_addr.s_addr = INADDR_ANY;
    localAddr.sin_port = htons(port);

    if (bind(localSocket, (struct sockaddr *)&localAddr, sizeof(localAddr)) < 0)
    {
        perror("Can't bind() socket");
        exit(1);
    }

    listen(localSocket, 3);

    std::cout << "Waiting for connections...\n"
              << "Server Port:" << port << std::endl;

    while (1)
    {
        remoteSocket = accept(localSocket, (struct sockaddr *)&remoteAddr, (socklen_t *)&addrLen);
        if (remoteSocket < 0)
        {
            perror("accept failed!");
            exit(1);
        }
        std::cout << "Connection accepted" << std::endl;
        socket_val = remoteSocket;
        break;
    }


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

void UVision::stream(){
  // send processed image
  Mat img, imgGray;
  img = Mat::zeros(720, 1280, CV_8UC1);

  cvtColor(frame_ud, imgGray, COLOR_BGR2GRAY);

  int imgSize = imgGray.total() * imgGray.elemSize();
  int bytes = 0;
  //int imgSize = frame.total() * frame.elemSize();
  if ((bytes = send(socket_val, imgGray.data, imgSize, 0)) < 0)
  {
      close(socket_val);
      std::cerr << "bytes = " << bytes << std::endl;
      
      exit(1);
      //break;
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
  int min_radio = 30;

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

  cv::cvtColor(frame_ud, frame_HSV, COLOR_BGR2HSV);
  //cv::blur(frame_HSV, frame_HSV, cv::Size(7, 7));
  cv::inRange(frame_HSV, low_orange, up_orange ,frame_threshold);
  cv::erode(frame_threshold, frame_ed, cv::Mat(), cv::Point(-1,-1), 1);
  cv::dilate(frame_ed, frame_ed, cv::Mat(), cv::Point(-1,-1), 1);
  //frame_ud.copyTo(frame_prep, frame_ed);
  if (showImage)
  {
    imshow("THR", frame_ed);
  }


  // cv::cvtColor(frame_prep, frame_gray, COLOR_BGR2GRAY);
  // cv::GaussianBlur(frame_gray, frame_gray, Size(9, 9), 0);

  vector<Vec3f> circles;
  // cv::HoughCircles(frame_gray, circles, HOUGH_GRADIENT, 1,
  //             frame_gray.rows/4,  // change this value to detect circles with different distances to each other
  //             200, 5, 40, 100 // change the last two parameters
  //         // (min_radius & max_radius) to detect larger circles
  // );
  cv::HoughCircles(frame_ed, circles, HOUGH_GRADIENT, 1.15,
              frame_ed.rows/4,  // change this value to detect circles with different distances to each other
              85, 15, min_radio, 100 // change the last two parameters
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
          cv::circle( frame_ud, center, 1, Scalar(0,255,0), 1, LINE_AA);
          // circle outline
          int radius = c[2];
          cv::circle( frame_ud, center, radius, Scalar(255,0,255), 1, LINE_AA);
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
  cout << "# LOOP FRAMES FOR " << obj << "\n";

  vector<cv::Mat1f> samples;
  cv::Mat1f accumulated = cv::Mat1f::zeros(1, 3);  // Accumulated sum
  int numSamples = 0;


  UTime t;
  t.now();
  frameSerial = 0;
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
      
      if (streaming)
      {
        stream();

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
      cout << "distance : " << distance << "\n";
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
  cout << "Samples " << filteredSamples.size() << std::endl;

  if (filteredSamples.size()>12){
    cout << "FOUND "<< obj << std::endl;
    
    return true;

  }else{
    cout << "NOT FOUND "<< obj << std::endl;

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


void UVision::move(cv::Mat1f position, string mode="MOVING"){
  const int MSL = 200;
  char s[MSL];
  double radians = atan(position.at<float>(0, 1) / position.at<float>(0, 0));
  double degrees = radians * (180.0 / M_PI);
  float dist = sqrt(pow(position.at<float>(0, 1),2) + pow(position.at<float>(0, 0),2)) - arm_dist;

  float vel = 0.1;
  if (mode=="backward"){
    dist = -dist;
    vel = -0.1;
    degrees = 0; 
  }

  float x = dist*cos(radians);
  float y = dist*sin(radians);

  if (true){

    update_robot_pos(x, y, degrees);
    //float x_p = 

    std::cout << "Angle " << degrees << " degrees " << " Distance: "<< dist << std::endl;
    sound.say(mode.c_str(), 0.3);
    // remove old mission
    bridge.tx("regbot mclear\n");
    // clear events received from last mission
    event.clearEvents();
    //usleep(2000);

    bridge.tx("regbot madd vel=0.0, log=3.0: time=0.02\n");

    snprintf(s,MSL,"regbot madd vel=%.2f,tr=0.0:turn=%.1f\n", 0.1, degrees);
    bridge.tx(s);
    std::cout << s << std::endl;

    bridge.tx("regbot madd vel=0.0: time=0.08\n");

    snprintf(s,MSL,"regbot madd vel=%.2f:dist=%.2f\n", vel, dist);
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


  robot_angle = robot_angle-180;
  std::cout << "Position of origin: (" << x << ", " << y << ")" << std::endl;
  std::cout << "robot_angle!!: " << robot_angle << std::endl;
  
  //update_robot_pos(x, y, robot_angle);

  const int MSL = 200;
  char s[MSL];

  // cv::Mat1f new_pos = rotate_translate(robot_pos, robot_angle);
  // move(new_pos);
  std::cout << "Angle " << robot_angle << " degrees " << " Distance: "<< dist << std::endl;
  sound.say("HOME", 0.3);
  // remove old mission
  bridge.tx("regbot mclear\n");
  // clear events received from last mission
  event.clearEvents();
  //usleep(2000);

  bridge.tx("regbot madd vel=0.0, log=3.0: time=0.02\n");

  snprintf(s,MSL,"regbot madd vel=%.2f,tr=0.0:turn=%.1f\n", 0.2, robot_angle);
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
  bridge.tx("regbot madd servo=1,pservo=-100,vservo=120:time=8\n");
  bridge.tx("regbot madd servo=2,pservo=-600:time=2\n");
  bridge.tx("regbot madd servo=1,pservo=600,vservo=120:time=8\n");

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
 
  bridge.tx("regbot madd servo=1,pservo=-100,vservo=120:time=8\n");
  bridge.tx("regbot madd servo=2,pservo=100:time=2\n");
  bridge.tx("regbot madd servo=1,pservo=600,vservo=120:time=8\n");

  bridge.tx("regbot start\n");
  cout << "Releasing a ball...\n";
  event.waitForEvent(0); 

}

void UVision::prepareArmAruco(){
  //servo=1,pservo=-600, vservo=120:time=8

  bridge.tx("regbot mclear\n");
  // clear events received from last mission
  event.clearEvents();
  //usleep(2000);
 
  bridge.tx("regbot madd servo=1,pservo=50,vservo=120:time=8\n");
  bridge.tx("regbot madd servo=2,pservo=325:time=2\n");

  bridge.tx("regbot start\n");
  cout << "prepare arm...\n";
  event.waitForEvent(0); 
}

void UVision::takeAruco(){
  //servo=1,pservo=-600, vservo=120:time=8

  bridge.tx("regbot mclear\n");
  // clear events received from last mission
  event.clearEvents();
  //usleep(2000);
 
  bridge.tx("regbot madd servo=2,pservo=-500:time=2\n");
  bridge.tx("regbot madd servo=1,pservo=600,vservo=120:time=8\n");

  bridge.tx("regbot start\n");
  cout << "Taking a ARUCO...\n";
  event.waitForEvent(0); 
}

void UVision::releaseAruco(){
  //servo=1,pservo=-600, vservo=120:time=8

  bridge.tx("regbot mclear\n");
  // clear events received from last mission
  event.clearEvents();
  //usleep(2000);
 
  bridge.tx("regbot madd servo=1,pservo=50,vservo=120:time=8\n");
  bridge.tx("regbot madd servo=2,pservo=325:time=2\n");

  bridge.tx("regbot start\n");
  cout << "Release Aruco...\n";
  event.waitForEvent(0); 
}

void UVision::turn_angle(float angle, float vel, float tr = 0.0){
  const int MSL = 200;
  char s[MSL];
  sound.say("HOME", 0.3);
  bridge.tx("regbot mclear\n");
  event.clearEvents();

  bridge.tx("regbot madd vel=0.0, log=3.0: time=0.02\n");

  snprintf(s,MSL,"regbot madd vel=%.2f,tr=%.2f:turn=%.2f\n", vel,tr, angle);
  bridge.tx(s);
  std::cout << s << std::endl;
  bridge.tx("regbot madd vel=0.0: time=0.08\n");

  bridge.tx("regbot start\n");
  cout << "turn angle..."<< angle << "\n";
  event.waitForEvent(0); 

}

void UVision::goto_aruco_area(){
  sound.say("ARUCO", 0.3);
  bridge.tx("regbot mclear\n");
  event.clearEvents();

  bridge.tx("regbot madd vel=0.0, log=3.0: time=0.02\n");
  bridge.tx("regbot madd servo=1,pservo=600,vservo=120:time=6\n");
  bridge.tx("regbot madd vel=0.2,edger=0.0,white=1:dist=1.0\n");
  bridge.tx("regbot madd vel=0.2,edger=0.0,white=1: lv<3,xl>16\n");
  bridge.tx("regbot madd vel=0.2: dist=0.5\n");
  bridge.tx("regbot madd vel=0.2, tr=0.1: turn=-90\n"); 
  bridge.tx("regbot madd vel=0.2: xl>16\n");
  bridge.tx("regbot madd vel=0.1, tr=0.1: turn=-90\n"); 
  bridge.tx("regbot madd vel=0.1,edger=0.0,white=1: lv<3,xl>16\n");
  bridge.tx("regbot madd vel=0.1, tr=0.1: turn=-90\n"); 
  bridge.tx("regbot madd vel=0.1,edger=0.0,white=1: lv<3,dist=0.15\n");
  bridge.tx("regbot madd vel=-0.2: dist=-0.60\n"); 
  bridge.tx("regbot madd vel=0.2: dist=0.20\n"); 
  bridge.tx("regbot madd vel=0.0:time=0.8\n");

  bridge.tx("regbot start\n");
  cout << "GOTO ARUCO..." << "\n";
  event.waitForEvent(0); 

}



void UVision::drive(float dist, float vel){
  const int MSL = 200;
  char s[MSL];
  sound.say("HOME", 0.3);
  bridge.tx("regbot mclear\n");
  event.clearEvents();

  bridge.tx("regbot madd vel=0.0, log=3.0: time=0.02\n");

  snprintf(s,MSL,"regbot madd vel=%.2f:dist=%.2f\n",vel, dist);
  bridge.tx(s);
  std::cout << s << std::endl;
  bridge.tx("regbot madd vel=0.0: time=0.02\n");

  bridge.tx("regbot start\n");
  cout << "drive...\n";
  event.waitForEvent(0); 

}

void UVision::execute_instruction(string instruction){
  bridge.tx("regbot mclear\n");
  event.clearEvents();

  bridge.tx("regbot madd vel=0.0, log=3.0: time=0.02\n");

  bridge.tx(instruction.c_str());
  std::cout << instruction << std::endl;
  bridge.tx("regbot madd vel=0.0: time=0.02\n");

  bridge.tx("regbot start\n");
  cout << "drive...\n";
  event.waitForEvent(0); 
}

void UVision::move_arround(){
  move_arround_dir = -1*move_arround_dir;

  cv::Vec3f vec3f(0.5f, move_arround_dir*0.2f, 0.0f);

  cv::Mat1f position = cv::Mat1f(vec3f);
  move(position, "MOVE ARROUND");
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
  move_arround_dir = -1;

  addBoundaryPoint(0.0, 1.0);
  addBoundaryPoint(5.0, 1.0);
  addBoundaryPoint(0.0, -3.0);
  addBoundaryPoint(5.0, -3.0);
  releaseBall();
  while(n<3 and t.getTimePassed() < 180){
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
        else {
          //MOVE ARROUND
          move_arround();


        }
      }
      else {
        //MOVE ARROUND
        move_arround();


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

cv::Mat1f UVision::aruco_location(){
  


}


bool UVision::doFindAruco(float seconds, int id=-1)
{ // image is in 'frame'
  printf("# GET ARUCO\n");
  markerSamples.clear();
  int samples_n = 5;
  bool status = false;

  Ptr<aruco::Dictionary> dictionary = cv::makePtr<aruco::Dictionary>(aruco::getPredefinedDictionary(cv::aruco::DICT_4X4_100));
  cv::Vec3i pos_in_frame(0, 0, 0);
  leftMarkerId = -1;
  minCornerX = std::numeric_limits<float>::max();
  // Detect the markers in the image
  std::vector<int> markerIds;
  std::vector<std::vector<Point2f>> markerCorners;


  //releaseBall();
  //cv::Mat1f position(1, 3);
  //position << 1.0f, 0.0f, 0.0f;
  //move(position);
  //turn_angle(-90);

  UTime t;
  t.now();

  while (t.getTimePassed()< 0.8){
    cout << "# wait for the first frames\n";
    getNewestFrame(); 

  }

  t.now();
  

  //while (t.getTimePassed() < seconds and camIsOpen and not terminate and n<5) {
  while (t.getTimePassed() < seconds and camIsOpen and not terminate) {
    //cap >> image;
    getNewestFrame(); 

    if (gotFrame){
      
      aruco::detectMarkers(frame_ud, dictionary, markerCorners, markerIds);
      
      // Draw the marker outlines on the image



      for (int i = 0; i < markerIds.size(); i++) {


        float width = cv::norm(markerCorners[i][3] - markerCorners[i][2]);
        float heigh = cv::norm(markerCorners[i][1] - markerCorners[i][2]);

        cout << "width " << width <<" heigh" << heigh << " dif " << abs(width-heigh) <<"\n";

        if (abs(width-heigh)<20){
          int markerId = markerIds[i];
          // Calculate the center of the marker
          cv::Point2f center(0.f, 0.f);
          for (const auto& corner : markerCorners[i]) {
            center += corner;
          }
          center /= 4.f;


          // Calculate the width of the marker
          //float markerWidth = cv::norm(markerCorners[i][3] - markerCorners[i][2]); // check which corners must be used
          std::cout << "Marker " << markerIds[i] << " detected. Width: " << width << " pixels. Corners: ";
          std::cout << "Center: " << center << std::endl;
          std::cout << "markerCorners[i][0]: " << markerCorners[i][0] << std::endl;

          cv::circle(frame_ud, markerCorners[i][1], 5, cv::Scalar(0,255,0));
          cv::circle(frame_ud, markerCorners[i][2], 5, cv::Scalar(0,255,0));

          pos_in_frame[0] = center.x;
          pos_in_frame[1] = center.y;
          pos_in_frame[2] = width/2;


          // Check if the marker id already exists in the dictionary
          if (markerSamples.find(markerId) != markerSamples.end()) {
              // If the marker id exists, add the current sample to the list of samples for this marker id
              markerSamples[markerId].push_back(pos_in_frame);
          } else {
              // If the marker id does not exist, create a new entry in the dictionary with the current marker id and the current sample as the first sample
              markerSamples[markerId] = {pos_in_frame};
          }

          float cornerX = markerCorners[i][0].x; // Get the x coordinate of the first corner point of the current marker

          if (id==-1){
            if (cornerX < minCornerX) { // Check if the x coordinate is smaller than the current minimum
                minCornerX = cornerX; // Update the minimum x coordinate
                leftMarkerId = markerIds[i]; // Update the left marker id
            }
          }else{
            leftMarkerId = id;
          }
          // aruco_pos = calc_pos3drob(pos_in_frame, aruco_size);
          // std::cout << "aruco_pos: " << aruco_pos << std::endl;
          // Draw a circle at the center of the marker
          cv::circle(frame_ud, center, 10, cv::Scalar(255,0,0));
        }

      }

      // 20 53 6
      if (showImage)
      {
        imshow("ARUCO", frame_ud);
      }
      if (streaming)
      {
        //int x = frame_ud.cols / 2;
        //cv::line(frame_ud, cv::Point(x, 0), cv::Point(x, frame_ud.rows), cv::Scalar(0, 255, 0), 1);

        stream();

      }

      waitKey(25);
    }
  }


  if (leftMarkerId > -1){
    //std::cout << "markerSamples " << markerSamples  << std::endl;
    auto it = markerSamples.find(leftMarkerId);
    if (it != markerSamples.end()) {
      int numSamples = it->second.size();
      std::cout << "Marker " << leftMarkerId << " has " << numSamples << " samples." << std::endl;
      if (numSamples> 0){ // this can be modify to get a robust position
        std::vector<cv::Vec3i> filteredSamples;
        auto& samples = markerSamples[leftMarkerId];
        
        cv::Vec3i meanpos;
        for (const auto& sample : samples) {
            meanpos += sample;
        }

        std::cout << "meanpos " << meanpos << std::endl;
        meanpos /= static_cast<float>(samples.size());
        std::cout << "meanpos post" << meanpos << std::endl;
        
        for (const auto& sample : samples) {
            // Calculate the Euclidean distance between the first and second points
            float dist = cv::norm(meanpos - sample);
            std::cout << "Distance " << dist << std::endl;
            
            // If the distance is less than the maximum distance, keep the sample
            if (dist < maxDist_aruco) {
                filteredSamples.push_back(sample);
            }
        }

        for (const auto& sample : filteredSamples) {
            leftArucoFramePos += sample;
        }
        leftArucoFramePos /= static_cast<float>(filteredSamples.size());
        std::cout << "filteredSamples " << filteredSamples.size() << std::endl;
        if (filteredSamples.size()>samples_n){
          status = true;

        }
      }
    } else {
        std::cout << "Marker " << leftMarkerId << " not found in markerSamples map." << std::endl;
    }
  } 
  
  if (status){
    std::cout << "leftArucoFramePos " << leftArucoFramePos <<  std::endl;
    aruco_pos = calc_pos3drob(leftArucoFramePos, aruco_size);
    aruco_pos(0, 0) = aruco_pos.at<float>(0, 0) - 0.02;
    //aruco_pos(0, 1) = aruco_pos.at<float>(0, 1) + 0.015;

    std::cout << "aruco_pos " << aruco_pos <<  std::endl;
  }

  return status;
}

bool UVision::aruco_mission(float seconds){
  UTime t;

  //turn_angle(-90.0, 0.2);
  goto_aruco_area();

  t.now();
  int n=0;
  string inst;

  while(t.getTimePassed() < seconds and n<4){ // add number of completed arucos
    bool status = doFindAruco(15);
    if (false){
      std::cout << "Marker: "<<  leftMarkerId << " aruco_pos: " << aruco_pos << std::endl;
      float x = aruco_pos.at<float>(0, 0);
      float dist = x - arm_dist; //- 0.05;
      //float theta = 7 * (M_PI/180); // compensation angle can be removed(it is to compensate the error related to the distances of the object)
      float y = aruco_pos.at<float>(0, 1) + 0.025; // 0.01 is the offset of the arm
      //y = y - dist*sin(theta) * y/abs(y) + 0.05; //  compensation of the error - dist*sin(theta) * y/abs(y) + 0.05
      std::cout << "y " << y <<  std::endl;
      std::cout << "dist " << dist <<  std::endl;
      

    }
    // prepareArmAruco();
    // takeAruco();
    
    
    if (status){
      std::cout << "Marker: "<<  leftMarkerId << " aruco_pos: " << aruco_pos << std::endl;
      float x = aruco_pos.at<float>(0, 0);
      float dist = x - arm_dist; //- 0.05;
      //float theta = 7 * (M_PI/180); // compensation angle can be removed(it is to compensate the error related to the distances of the object)
      float y = aruco_pos.at<float>(0, 1); // 0.01 is the offset of the arm
      //y = y - dist*sin(theta) * y/abs(y) + 0.05; //  compensation of the error - dist*sin(theta) * y/abs(y) + 0.05
      std::cout << "y " << y <<  std::endl;
      std::cout << "dist " << dist <<  std::endl;
      

      
      //if (y >= -0.04 && y <= 0.04){
      if (true){
        
        std::cout << "Y in interval " << y <<  std::endl;
        prepareArmAruco();

        //float angle = atan(y/dist)* (180.0 / M_PI);
        //turn_angle(-angle, 0.1);

        //move(aruco_pos);

        inst = "regbot madd vel=0.1,edger=0.0,white=1:dist=" + std::to_string(dist) + "\n";
        execute_instruction(inst);

        //drive(dist, 0.1);
        takeAruco();
          
        
        //drive(-0.8, -0.1);
        inst = "regbot madd vel=-0.2: xl>16\n";
        execute_instruction(inst);
        drive(-0.8, -0.1);
        turn_angle(90, 0.1);
        drive(-0.2, -0.1);
        status = doFindAruco(15, leftMarkerId);
        if (status){
          std::cout << "ARUCO HOME FOUND" << aruco_pos <<  std::endl;

          drive(-0.05, -0.1);


          inst = "regbot ctrn 1 1 1 1 9999.99 1 1 1 1 1 1 1 1 1 1 0 1 9999.99 1 0 1 0 1 1 1 9999.99\n";
          execute_instruction(inst);

          move(aruco_pos);

          inst = "regbot ctrn 0 1 0 1 9999.99 1 0 1 1 0 1 1 0 1 1 0 1 9999.99 1 0 1 0 1 1 0 9999.99\n";
          execute_instruction(inst);

          //drive(0.3, 0.1);
          releaseAruco();
          //drive(-0.3, -0.1);
          //RETURN TO THE ARUCOS INITAL POS
          move(aruco_pos,"backward");
          float x = aruco_pos.at<float>(0, 0);
          float y = aruco_pos.at<float>(0, 1);
          float angle = atan(y/x)* (180.0 / M_PI);
          turn_angle(-angle, 0.1);

        }else{
          std::cout << "ARUCO HOME NOT FOUND" <<  std::endl;
          turn_angle(90, 0.1);
          drive(0.70, 0.1); // needs also tunning
          turn_angle(-90, 0.1);
          drive(0.25, 0.1); // needs also tunning
          releaseAruco();
          drive(-0.4, -0.1);

        }
        drive(-0.3, -0.1);
        turn_angle(-90, 0.1);
        inst = "regbot madd vel=0.2: lv>3,xl>16\n";
        execute_instruction(inst);

// bridge.tx("regbot madd vel=0.2: xl>16\n");
// bridge.tx("regbot madd vel=0.1, tr=0.1: turn=-90\n"); 
// bridge.tx("regbot madd vel=0.1,edger=0.0,white=1: lv<3,xl>16\n");
// bridge.tx("regbot madd vel=0.1, tr=0.1: turn=-90\n"); 
// bridge.tx("regbot madd vel=0.1,edger=0.0,white=1: lv<3,dist=0.15\n");

        bridge.tx("regbot madd servo=1,pservo=600,vservo=120:time=6\n");
        
        inst = "regbot madd servo=1,pservo=600,vservo=120:time=6\n";
        execute_instruction(inst);
        inst = "regbot madd vel=0.2: lv>3,xl>16\n";
        execute_instruction(inst);
        inst = "regbot madd vel=0.1, tr=0.1: turn=90\n";
        execute_instruction(inst);
        inst = "regbot madd vel=0.1,edger=0.0,white=1: lv<3,xl>16\n";
        execute_instruction(inst);
        inst = "regbot madd vel=0.1, tr=0.1: turn=-90\n";
        execute_instruction(inst);
        inst = "regbot madd vel=0.1,edger=0.0,white=1: lv<3,dist=0.15\n";
        execute_instruction(inst);
        inst = "regbot madd vel=-0.2: dist=-0.60\n";
        execute_instruction(inst);
        inst = "regbot madd vel=0.2: dist=0.20\n";
        execute_instruction(inst);

        n++;
        float di = n*0.15;

        inst = "regbot madd vel=0.1,edger=0.0,white=1:dist=" + std::to_string(di) + "\n";
        execute_instruction(inst);

        // inst = "regbot madd vel=-0.2: dist=-0.15\n";
        // execute_instruction(inst);
          
          //move until find the line
        
      }
      // else{
      //   std::cout << "Y not in interval " << y <<  std::endl;
      //   if (dist<0.1){
      //     std::cout << "Small dist " << dist <<  std::endl;
      //     drive(-0.1, -0.1);
      //   }
      //   //move(aruco_pos);
      //   if (y>0){turn_angle(90, 0.1);}
      //   else{turn_angle(-90, -0.1);}
      //   drive(y, 0.1);
      //   if (y>0){turn_angle(-90, 0.1);}
      //   else{turn_angle(90, 0.1);}
      //   drive(dist-0.05, 0.1);
      // }
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