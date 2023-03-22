/*  
 
 Copyright © 2022 DTU, Christian Andersen jcan@dtu.dk
 
 The MIT License (MIT)  https://mit-license.org/
 
 Permission is hereby granted, free of charge, to any person obtaining a copy of this software
 and associated documentation files (the “Software”), to deal in the Software without restriction, 
 including without limitation the rights to use, copy, modify, merge, publish, distribute, 
 sublicense, and/or sell copies of the Software, and to permit persons to whom the Software 
 is furnished to do so, subject to the following conditions:
 
 The above copyright notice and this permission notice shall be included in all copies 
 or substantial portions of the Software.
 
 THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
 INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
 PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE 
 FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, 
 ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN 
 THE SOFTWARE. */

/*aiddsda*/

#include <iostream>
#include "src/ubridge.h"
#include "src/uvision.h"
#include "src/upose.h"
#include "src/ucomment.h"
#include "src/ustate.h"
#include "src/uplay.h"
#include "src/uevent.h"

// to avoid writing std:: 
using namespace std;


void setup(int argc, char **argv)
{ // check for command line parameters
  for (int i = 1; i < argc; i++)
  { // check for command line parameters
    // for process debug
    if (strcmp(argv[i], "help") == 0)
    { 
      printf("-----\n# User mission command line help\n");
      printf("# usage:\n#   ./user_mission [help] [ball] [show] [aruco] [videoX]\n-----\n");
      return;
    }
  }
  // connect to robot hardware using bridge
  bridge.setup("127.0.0.1", "24001", argc, argv);
  if (true or bridge.connected)
  {  /// call setup for data structures
    pose.setup();
    comment.setup();
    state.setup();
    vision.setup(argc, argv);
    event.setup();
    printf("# Setup finished OK\n");
  }
  else
    printf("# Setup failed\n");
  sound.say("me?... I am a depressed robot... help me.", 0.1);
  while (sound.isSaying())
    usleep(100000);
}



void step2()
{
  sound.say("There is no step 2.", 0.3);
  cout << "There is no step 2 yet\n";
}

void moveRobot()
{
  const int MSL = 200;
  char s[MSL];
  float a=0.4, b=3;
  snprintf(s,MSL,"regbot madd vel=%f: time=%f\n", a, b);
  //bridge.tx(s);
  
}

void ballTrack(cv::Mat1f ballPos)
{
  
  float arm_dist = 0.35;
  const int MSL = 200;
  char s[MSL];
  double radians = atan(ballPos.at<float>(0, 1) / ballPos.at<float>(0, 0));
  double degrees = radians * (180.0 / M_PI);
  std::cout << "Angle " << degrees << " degrees" << std::endl;
  float dist = ballPos.at<float>(0, 0) - arm_dist;


  sound.say(". Step one.", 0.3);
  // remove old mission
  bridge.tx("regbot mclear\n");
  // clear events received from last mission
  event.clearEvents();

  //bridge.tx("regbot madd vel=0.0, log=3.0: time=0.02 \n");
  //vel=0.3,tr=0.2:turn=90
  snprintf(s,MSL,"regbot madd vel=%f,tr=0.2:turn=%f\n", 0.2, degrees);
  bridge.tx(s);

  //snprintf(s,MSL,"regbot madd vel=%f: time=%f\n", a, b);
  //vel=5.0: dist=30.0 
  snprintf(s,MSL,"regbot madd vel=%f:dist=%f\n", 0.2, dist);
  bridge.tx(s);


  // start this mission
  bridge.tx("regbot start\n");
  // wait until finished
  //
  cout << "Waiting for step 1 to finish (event 0 is send, when mission is finished)\n";
  event.waitForEvent(0);
//   sound.say(". Step one finished.");
  
}

int main(int argc, char **argv) 
{
  std::cout << "# Hello, Robobot user mission starting ...\n";
  setup(argc, argv);
  //
  //step1();
  //step1();
  //step2();
  //vision.processImage(60);
  int n = 1;
  UTime t;
  t.now();

  while(n<6 and t.getTimePassed() < 60){
    bool ball = vision.loopVideo(10);
    if (ball == true){
      std::cout << "# BALL FOUND "<< n <<" Pos: " << vision.ballPossition << "\n";
      ballTrack(vision.ballPossition);
      n +=1;
    }
  }
  //
  std::cout << "# Robobot user mission finished ...\n";
  // remember to close camera
  vision.stop();
  sound.say("I am finished... sorry danish.", 0.2);
  while (sound.isSaying())
    sleep(1);
  bridge.tx("regbot mute 1\n");
  return 0;
}