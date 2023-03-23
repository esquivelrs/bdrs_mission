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
#include <opencv2/imgproc.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core.hpp>
#include <opencv2/aruco.hpp>
#include <iostream>
#include <iostream>
#include<iostream>
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

  cv::Mat camera_matrix, dist_coeffs;

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

bool UVision::processImage(float seconds)
{ // process images in 'seconds' seconds
  UTime t, t2, t3, t4; // for timing
  t.now();
  int n = 0;
  int frameCnt = 0;
  float frameSampleTime = 1.5; // seconds
  while (t.getTimePassed() < seconds and camIsOpen and not terminate and n < 5)
  { // skip the first 20 frames to allow auto-illumination to work
    if (t4.getTimePassed() > frameSampleTime and frameSerial > 20)
    { // do every 1.5 second (or sample time)
      t4.now();
      getNewestFrame();    
      if (gotFrame)
      { // process
        printf("# got frame %d (%d) t=%.3f sec, dt=%.3f sec, size %dx%d\n", frameCnt, frameSerial, t.getTimePassed(), t2.getTimePassed(), frame.rows, frame.cols);
        t2.now();
        if (showImage)
        {
          t3.now();
          cv::imshow("raw image", frame);
          printf("Image show call took %.3f sec\n", t3.getTimePassed());
          cv::waitKey(300);
        }
        if (saveImage)
        { // save the image - with a number
          const int MSL = 100;
          char s[MSL];
          snprintf(s, MSL, "sandberg_%03d.png", n);
          t3.now();
          cv::imwrite(s, frame);
          printf("Image save took %.3f sec\n", t3.getTimePassed());
        }
        if (findBall and n > 2)
        {
          t3.now();
          ballBoundingBox.clear();
          terminate = doFindBall();
          printf("Find ball took %.3f sec\n", t3.getTimePassed());
          if (ballBoundingBox.size() >= 1)
          { // test if the ball is on the floor
            ballProjectionAndTest();
          }
        }
        frameCnt++;
      }
      n++;
    }
    else
      usleep(5000);
  }
  printf("# Ending vision loop (terminate=%d, camIsOpen=%d, n=%d\n", terminate, camIsOpen, n);
  return terminate or not camIsOpen;
}

int UVision::uvDistance(cv::Vec3b pix, cv::Vec3b col)
{ /// format is Y,V,U and Y is not used
  /// returns 0 for total match
  /// returns (block) distance in U,V space between pixel (pix) and color (col)
  /// returns at maximum 255
  int d = abs(pix[1] - col[1]) + abs(pix[2] - col[2]);
  if (d > 255)
    d = 255;
  else if (d < 0)
    d = 0;
  return d;
}


bool UVision::doFindBall()
{ // process pipeline to find
  // bounding boxes of balls with matched colour
  cv::Mat yuv;
  //cv::imwrite("rgb_balls_01.png", frame);
  cv::cvtColor(frame, yuv, cv::COLOR_BGR2YUV);
  int h = yuv.rows;
  int w = yuv.cols;
  //cv::imwrite("yuv_balls_01.png", yuv);
  printf("# YUV saved, size width=%d height=%d\n", w, h);
  //
  // color for filter
  cv::Vec3b yuvOrange = cv::Vec3b(128,88,187);
  cv::Mat gray1(h,w, CV_8UC1);
  // test all pixels
  for (int r = 0; r < h; r++)
  { // get pointers to pixel-row for destination image
    uchar * pOra = (uchar*) gray1.ptr(r); // gray
    for (int c = 0; c < w; c++)
    { // go through all pixels in this row
      int d;
      cv::Vec3b p = yuv.at<cv::Vec3b>(r,c);
      d = uvDistance(p, yuvOrange);
      *pOra = 255 - d;
      pOra++; // increase to next destination pixel
    }
  }
  cv::Mat gray2;
  cv::threshold(gray1, gray2, 230, 255, 3);
  //
  // remove small items with a erode/delate
  // last parameter is iterations, and could be increased
  cv::Mat gray3, gray4;
  cv::erode(gray2, gray3, cv::Mat(), cv::Point(-1,-1), 2);
  cv::dilate(gray3, gray4, cv::Mat(), cv::Point(-1,-1), 2);
  if (showImage)
  { // show eroded/dilated image
    cv::imshow("Thresholede image", gray2);
    cv::imshow("Eroded/dilated image", gray4);
    cv::waitKey(25); // 1 second
  }
  //
  // find contours for further validation
  vector<vector<cv::Point> > contours;
  vector<cv::Vec4i> hierarchy; // not used, but needed
  cv::findContours( gray4, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE );
  if (showImage)
  { // show the found contours for debug
    cv::RNG rng(12345);
    cv::Mat col4 = cv::Mat::zeros( gray4.size(), CV_8UC3 );
    for( size_t i = 0; i< contours.size(); i++ )
    {
      cv::Scalar color = cv::Scalar( rng.uniform(0, 256), rng.uniform(0,256), rng.uniform(0,256) );
      cv::drawContours( col4, contours, (int)i, color, 2, cv::LINE_8, hierarchy, 0 );
    }
    imshow( "Contours", col4);
    cv::waitKey(10);
  }
  // Test for valid contours
  if (showImage)
    frame.copyTo(debugImg); // make copy of original image
  // iterate all contours
  for (int i = 0; i < (int)contours.size(); i++)
  {
    const vector<cv::Point>& ct = contours[i];
    printf("# Contour %d has %d points\n", i, (int)ct.size());
    // find boundingRect
    cv::Rect bb = cv::boundingRect(ct);
    // find longest side of bounding box
    int mx = bb.height;
    if (bb.width > mx)
      mx = bb.width;
    // half width
    int mx2 = mx/2;
    // filter for height and width should be fairly ewual - no more than 33% difference
    // the bounding box should be bigger than 13x13 pixels (area > 200 pixels)
    // the size should be less than 150 pixels
    if (abs(bb.height - bb.width) < mx / 3 and bb.area() > 200 and mx < 150)
    { // circle is OK so far
      // test if content is the right color too
      int okPixels = 0;
      int usedPixelCnt = 0;
      printf("# Object %d at %d,%d and width=%d, height=%d passed the size criteria\n", i, bb.x, bb.y, bb.width, bb.height);
      for (int r = 0; r < bb.height; r++)
      {
        uchar * pix = (uchar*) gray4.ptr(r + bb.y); // gray
        pix += bb.x;
        for (int c = 0; c < bb.width; c++)
        { // count pixels with right colour not from a circle
          // but use of a diamond shaped area is faster:
          // the sum of row and column distance from center 
          // should be less than half width of bounding box
          if ((abs(c - mx2) + abs(r - mx2)) < mx2)
          { // this should be inside the ball, and the colour counts
            okPixels += *pix; // add the pixel value (230..255, or 0 if not right)
            pix++; // move to next pixel
            usedPixelCnt++; // count the pixel for average
          }
        }
      }
      float avg = (float)okPixels / float(usedPixelCnt*255);
      printf("#  -- has %d of %d as the right color, average is %g\n", okPixels, (int)bb.area(), avg);
      // filter on average color inside bounding box diamond (more than 60% has OK color)
      if (avg > 0.6)
      { // should be counted as OK, add to list of bounding boxes
        ballBoundingBox.push_back(bb);
        // draw the box on the color image
        if (showImage)
          cv::rectangle(debugImg, bb.tl(), bb.br(), cv::Vec3b(230,0,155), 2 );
      }
    }
  }
  printf("Found %d/%d balls filtered for size and average color\n", 
         (int)ballBoundingBox.size(), (int)contours.size());
  if (showImage)
  {
    imshow("Debug image", debugImg);  
  }
  return true;
}


void UVision::ballProjectionAndTest()
{
  bool done = ballBoundingBox.size() == 0;
  if (not done)
  {
    for (int i = 0; i < (int)ballBoundingBox.size(); i++)
    {
      printf("---\n");
      cv::Rect bb = ballBoundingBox[i];
      float diaPix = std::max(bb.width, bb.height);
      /// use focal length to find distance
      //       diaPix    golfDia
      //       ------ = --------
      //          f        x
      // f = focal length, x = distance to ball
      float dist = golfBallDiameter * focalLength / float(diaPix);
      // the position in x (right) and y (down)
      float bbCenter[2] = {bb.x + bb.width/2.0f, bb.y + bb.height/2.0f};
      float frameCenter[2] = {frame.cols/2.0f, frame.rows/2.0f};
      // distance right of image center line - in meters
      float x = (bbCenter[0] - frameCenter[0])/focalLength * dist;
      // distance below image center line - in meters
      float y = (bbCenter[1] - frameCenter[1])/focalLength * dist;
      // make a vector of ball center with (x=forward, y=left, z=up)
      cv::Vec4f pos3dcam(dist, -x, -y, 1.0f);
      printf("# ball %d position in cam   coordinates (x,y,z)=(%.2f, %.2f, %.2f)\n", i, 
             pos3dcam[0], pos3dcam[1], pos3dcam[2]);
      // print used matrices and vector
      //  cout << "camToRobot: " << camToRobot << "\n";
      //  cout << "# pos3dcam  : " << pos3dcam << "\n";
      cv::Mat1f pos3drob = camToRobot * pos3dcam;
      printf("# ball %d position in robot coordinates (x,y,z)=(%.2f, %.2f, %.2f)\n", i, 
             pos3drob.at<float>(0), pos3drob.at<float>(1), pos3drob.at<float>(2));
      //
      if (showImage)
      { // put coordinates in debug image
        const int MSL = 100;
        char s[MSL];
        snprintf(s, MSL, "Ball %d at x=%.2f, y=%.2f, z=%.2f\n", i, pos3drob.at<float>(0), pos3drob.at<float>(1), pos3drob.at<float>(2));
        cv::putText(debugImg, s, cv::Point(bbCenter[0], bbCenter[1]), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 156));
        //
        imshow("Debug image", debugImg);
        if (saveImage)
          cv::imwrite("annotated_debug_image.jpg", debugImg);          
      }
    }
  }
  if (showImage)
  {
    printf("# projection done - press key to finish\n");
    cv::waitKey(0);
  }
}


bool UVision::getball(){
  Vec3i max_circle(0,0,0);
  Mat frame_HSV, frame_gray, frame_threshold, frame_ed;
  cv::cvtColor(frame, frame_gray, COLOR_BGR2GRAY);
  cv::GaussianBlur(frame_gray, frame_gray, Size(7, 7), 0);

  vector<Vec3f> circles;
  cv::HoughCircles(frame_gray, circles, HOUGH_GRADIENT, 1,
              frame_gray.rows/4,  // change this value to detect circles with different distances to each other
              200, 5, 40, 100 // change the last two parameters
          // (min_radius & max_radius) to detect larger circles
  );
  for( size_t i = 0; i < circles.size(); i++ )
  {
      Vec3i c = circles[i];
      Point center = Point(c[0], c[1]);
      // circle center


      if (0<(c[0]-c[2]/4) && (c[0]+c[2]/4)<frame.cols && 0<(c[1]-c[2]/4) && (c[1]+c[2]/4)<frame.rows){
        
        cv::cvtColor(frame, frame_HSV, COLOR_BGR2HSV);
        cv::Mat roi = frame_HSV(cv::Range(c[1]-c[2]/4, c[1]+c[2]/4), cv::Range(c[0]-c[2]/4, c[0]+c[2]/4));
        cv::Mat1b mask(roi.rows, roi.cols);
        cv::Scalar mean = cv::mean(roi, mask);
        int hue = round(mean[0]);
        int sat = round(mean[1]);
        int val = round(mean[2]);
        cout << "CIRCLE FOUND: " << center << " radio: " << c[2] << " hue: " << hue << " sat: " << sat << " val: " << val <<"\n";

        if (hue< 40 and val>200){
          cout << "BALL FOUND: " << center;
          cout << " radio: " << c[2] << " hue: " << hue << "\n";
          cv::circle( frame, center, 1, Scalar(0,100,100), 3, LINE_AA);
          // circle outline
          int radius = c[2];
          cv::circle( frame, center, radius, Scalar(255,0,255), 3, LINE_AA);
          std::string text = "BALL FOUND: (" + std::to_string(center.x) + ", " + std::to_string(center.y) + ")" + " radio: " + std::to_string(c[2]) + "hsv" + std::to_string(hue) + "," + std::to_string(sat)+ "," + std::to_string(val); 
          cv::putText(frame, text, center, cv::FONT_HERSHEY_DUPLEX, 0.4, cv::Scalar(0,0,0), 2, false);
          if (max_circle[2]<radius){
            max_circle[0] = c[0];
            max_circle[1] = c[1];
            max_circle[2] = radius;

          }


        }
      }
  }

  bool ball= false;
  if (0<max_circle[2]){
      cout << "TRACKING BALL: " << max_circle;
      int radio = max_circle[2];
      float dist = golfBallDiameter * focalLength / float(radio*2);
      // the position in x (right) and y (down)

      float bbCenter[2] = {max_circle[0], max_circle[1]};
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
      pos3drob = camToRobot * pos3dcam;
      ball = true;
            
  }
  //Vec3i max_circle(0,0,0);
  return ball;

}


float euclideanDistance(const cv::Mat1f& v1, const cv::Mat1f& v2) {
    cv::Mat1f diff = v1 - v2;
    return cv::sqrt(diff.dot(diff));
}

bool UVision::get_ball(float seconds)
{
  printf("# GET BALL\n");
  float distanceThreshold = 5.0;
  vector<cv::Mat1f> samples;
  cv::Mat1f accumulated = cv::Mat1f::zeros(1, 3);  // Accumulated sum
  int numSamples = 0;

  // VideoCapture cap(0);
  // if (!cap.isOpened()) {
  // cout << "cannot open camera";
  // }

  UTime t;
  t.now();

  while (t.getTimePassed()< 1){
    cout << "# wait for the first frames\n";
    getNewestFrame(); 

  }

  t.now();


  //while (t.getTimePassed() < seconds and camIsOpen and not terminate and n<5) {
  while (t.getTimePassed() < seconds and camIsOpen and not terminate) {
    //cap >> image;

    getNewestFrame(); 


    if (gotFrame){

      bool ball = getball();
      if (ball==true){

        printf("# ball %d position in robot coordinates (x,y,z)=(%.2f, %.2f, %.2f)\n", 
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
        imshow("BALLS", frame);
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
      if (distance <= distanceThreshold) {
          filteredSamples.push_back(sample);
      }
  }

  // Recalculate mean using only filtered samples
  cv::Mat1f filteredAccumulated = cv::Mat1f::zeros(1, 3);
  for (auto& sample : filteredSamples) {
      filteredAccumulated += sample;
  }
  ballPossition = filteredAccumulated / static_cast<float>(filteredSamples.size());

  // Print mean
  cout << "Mean: " << ballPossition << std::endl;
  cout << "Samples " << samples.size() << std::endl;

  if (samples.size()>5){
    
    return true;

  }else{

    return false;

  }



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
  Ptr<aruco::Dictionary> dictionary = cv::makePtr<aruco::Dictionary>(aruco::getPredefinedDictionary(cv::aruco::DICT_5X5_100));



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
