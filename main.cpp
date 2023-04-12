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

#include <string>
#include <iostream>
#include "src/ubridge.h"
#include "src/uvision.h"
#include "src/upose.h"
#include "src/ucomment.h"
#include "src/ustate.h"
#include "src/uplay.h"
#include "src/uevent.h"
#include "src/uencoder.h"
#include "src/utime.h"
#include <math.h>

using namespace std;

void setup(int argc, char **argv)
{ // check for command line parameters
  for (int i = 1; i < argc; i++)
  { // check for command line parameters for process debug
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
  {  // call setup for data structures
    pose.setup();
    comment.setup();
    state.setup();
    vision.setup(argc, argv);
    event.setup();
    enc.setup();
    sound.say("Set up complete", 0.2);
    printf("# Set up complete OK\n");
  }
  else
  {
    printf("# Setup failed\n");
    sound.say("Set up failed", 0.2);
  }
  while (sound.isSaying())
    usleep(100000);
  // sound.play("/home/local/Music/music.mp3", 0.05); // a bit of background music
}

void testMethod()
{
  bridge.tx("regbot mclear\n");
  event.clearEvents();

  // move forward for 3s
  // bridge.tx("regbot madd vel=0.2: time=3.0\n");
  
  // reverse and return to line
  bridge.tx("regbot madd vel=-0.3: lv=20, dist=1.0\n");
  bridge.tx("regbot madd vel=-0.3, tr=0.8: turn=110\n");
  bridge.tx("regbot madd vel=0.0: time=0.8\n");

  // start this mission
  bridge.tx("regbot start\n");
  cout << "user:# Waiting for " + string(__func__) + " to finish" << endl;
  event.waitForEvent(0);
  cout << "user:# obstacle " + string(__func__) + " has finished" << endl;
}

void guillotine()
{
  bridge.tx("regbot mclear\n");
  event.clearEvents();
  usleep(200);

  bridge.tx("regbot madd servo=1, pservo=700, vservo=150: time=3\n");   //set arm to top position
  bridge.tx("regbot madd vel=0.3: dist=0.3\n");
  bridge.tx("regbot madd vel=0.4: dist=0.8, lv=20\n");
  bridge.tx("regbot madd vel=0.4, edger=0.0, white=1: dist=3.6, lv<3\n"); // ends slightly after y-junction

  // start this mission
  cout << "user:# Waiting for " + string(__func__) + " to finish" << endl;
  bridge.tx("regbot start\n");
  event.waitForEvent(0);
  cout << "user:# obstacle " + string(__func__) + " has finished" << endl;
}

void ramp()
{
  bridge.tx("regbot mclear\n");
  event.clearEvents();
  usleep(2000);

  bridge.tx("regbot madd vel=0.5: dist=0.5, lv=20\n");
  bridge.tx("regbot madd vel=0.5, edger=0.0, white=1: dist=11.0\n"); // Drive till end of ramp
  bridge.tx("regbot madd vel=0.0: time=0\n");

  // complete the downRamp
  bridge.tx("regbot madd vel=0.2, edger=0.0, white=1: ir2<0.4\n");
  bridge.tx("regbot madd vel=0.2, tr=0.0: turn=180\n");
  bridge.tx("regbot madd vel=0.2, edger=0, white=1: lv<4, dist=1.1\n");


  // start this mission
  bridge.tx("regbot start\n");
  cout << "user:# Waiting for " + string(__func__) + " to finish" << endl;
  event.waitForEvent(0);
  cout << "user:# obstacle " + string(__func__) + " has finished" << endl;
}

void restartRamp()
{
  // from home position to midt ramp up turning upwards
  // travel from home1 to home2

  bridge.tx("regbot mclear\n");
  event.clearEvents();
  usleep(2000);

  bridge.tx("regbot madd servo=1, pservo=700, vservo=200: time=5.0\n");   // Raise arm
  bridge.tx("regbot madd vel=0.2: lv=20, dist=0.8\n");
  bridge.tx("regbot madd vel=0.3, edger=0.0, white=1: dist=1.0, xl>16  \n");  // find home1
  bridge.tx("regbot madd vel=0.2, tr=0.0: turn=200, time=3.0 \n");   // U-turn at home1
  bridge.tx("regbot madd vel=0.0: time=1.0\n");
  bridge.tx("regbot madd vel=0.2: lv=20, dist=0.8\n");
  bridge.tx("regbot madd vel=0.3, edgel=0.0, white=1: ir2<0.4  \n");
  bridge.tx("regbot madd vel=0.2, tr=0.0: turn=180 \n");    // U-turn in front of goal
  bridge.tx("regbot madd vel=0.0: time=1.0\n");
  bridge.tx("regbot madd vel=0.4: lv=20, dist=0.8\n");
  bridge.tx("regbot madd vel=0.4, edgel=0.0, white=1: dist=7.5 \n");
  bridge.tx("regbot madd vel=0.0: time=1.0\n");
  bridge.tx("regbot madd vel=0.2, tr=0.0: turn=190 \n");    // U-turn at home2
  bridge.tx("regbot madd vel=0.0: time=1.0\n");
  bridge.tx("regbot madd vel=0.2: lv=20, dist=0.8\n");
  bridge.tx("regbot madd vel=0.2, edgel=0.0, white=1: dist=0.5  \n");

  
  // start this mission
  bridge.tx("regbot start\n");
  cout << "user:# Waiting for " + string(__func__) + " to finish\n" << endl;
  event.waitForEvent(0);
  cout <<  "user:# obstacle " + string(__func__) + " has finished\n" << endl;
}

void ballOnRamp()
{
  bridge.tx("regbot mclear\n");
  event.clearEvents();
  usleep(2000);
  bridge.tx("regbot madd servo=1, pservo=600, vservo=200: time=5.0\n");   // Raise arm
  bridge.tx("regbot madd servo=2, pservo=325: time=4.0\n");   // Open gripper

  // Go to XL
  bridge.tx("regbot madd vel=0.2: dist=0.1\n");
  bridge.tx("regbot madd vel=0.2: lv=20, dist=0.05\n"); 
  bridge.tx("regbot madd vel=0.2, edgel=0, white=1:lv<4, xl>15\n");

  // After XL
  bridge.tx("regbot madd vel=0.2: dist=0.1\n");
  bridge.tx("regbot madd vel=0.2: lv=20, dist=0.05\n"); 
  bridge.tx("regbot madd vel=0.2, edgel=0, white=1: dist=1.0\n");

  // Turn to get ball
  bridge.tx("regbot madd vel=0.05,tr=0.0:turn=90\n");
  bridge.tx("regbot madd vel=0.0: time=0.1\n");  // 

  // Grip ball
  bridge.tx("regbot madd servo=1, pservo=-50, vservo=120: time=8.0\n");   // Lower arm
  bridge.tx("regbot madd vel=0.05: dist=0.1, time=0.5\n");  //
  bridge.tx("regbot madd vel=0.0: time=0.1\n");  // 
  bridge.tx("regbot madd servo=2, pservo=-600: time=4.0\n");   // Close gripper
  bridge.tx("regbot madd servo=1, pservo=100, vservo=120: time=5.0\n");   // Lift arm so gripper can close properly

  // Turn back on track and position on ramp
  bridge.tx("regbot madd vel=0.05,tr=0.0:turn=90\n");

  bridge.tx("regbot madd vel=0.3:dist=0.1\n");
  bridge.tx("regbot madd vel=0.3: lv=20, dist=0.05\n"); 
  bridge.tx("regbot madd vel=0.3, edger=0, white=1:lv<4, xl>15\n");
  bridge.tx("regbot madd vel=0.3: lv=20, dist=0.05\n");
  bridge.tx("regbot madd vel=0.3, edger=0, white=1:dist=1.5\n");

  bridge.tx("regbot madd vel=0.1,tr=0.0:turn=180\n");

  bridge.tx("regbot start \n");

  event.waitForEvent(0);
	cout << "user:# Waiting  for " + string(__func__) + " to finish" << endl;

}

void stairs()
{
  bridge.tx("regbot mclear\n");
  event.clearEvents();
  usleep(2000);
  bridge.tx("regbot madd servo=1, pservo=200, vservo=200: time=5.0\n");   // Raise arm
  bridge.tx("regbot madd servo=2, pservo=100: time=4.0\n");   // Open gripper

  // Go to XL
  bridge.tx("regbot madd vel=0.2:dist=0.1\n");
  bridge.tx("regbot madd vel=0.2: lv=20, dist=0.05\n"); 
  bridge.tx("regbot madd vel=0.2, edgel=0, white=1:lv<4, xl>15\n");

  // After XL
  bridge.tx("regbot madd vel=0.2: dist=0.1\n");
  bridge.tx("regbot madd vel=0.2: lv=20, dist=0.05\n"); 
  bridge.tx("regbot madd vel=0.2, edgel=0, white=1: dist=1.47\n");

  // Go down stairs - first step
  bridge.tx("regbot madd vel=0.0:time=0.1\n");
  bridge.tx("regbot madd servo=1, pservo=-450, vservo=120: time=6.0\n");   // lower arm much
  bridge.tx("regbot madd vel=0.05:dist=0.2\n");
  bridge.tx("regbot madd vel=0.0:time=0.5\n");
  bridge.tx("regbot madd servo=1, pservo=300, vservo=100: time=10.0\n");   // raise arm
  bridge.tx("regbot madd vel=0.05:dist=0.1\n");

  bridge.tx("regbot madd vel=-0.1:time=1.5\n");
  bridge.tx("regbot madd vel=0.2: lv=20, dist=0.05\n");
  bridge.tx("regbot madd vel=0.2, edgel=0, white=1: dist=0.18\n");
  bridge.tx("regbot madd vel=0.0:time=0.5\n");


// Second step:
  bridge.tx("regbot madd servo=1, pservo=-450, vservo=120: time=6.0\n");   // lower arm much
  bridge.tx("regbot madd vel=0.05:dist=0.2\n");
  bridge.tx("regbot madd vel=0.0:time=0.5\n");
  bridge.tx("regbot madd servo=1, pservo=300, vservo=100: time=10.0\n");   // raise arm
  bridge.tx("regbot madd vel=0.05:dist=0.1\n");

  bridge.tx("regbot madd vel=-0.1:time=1.5\n");
  bridge.tx("regbot madd vel=0.2: lv=20, dist=0.05\n");
  bridge.tx("regbot madd vel=0.2, edgel=0, white=1: dist=0.18\n");
  bridge.tx("regbot madd vel=0.0:time=0.5\n");

// Third step:
  bridge.tx("regbot madd servo=1, pservo=-450, vservo=120: time=6.0\n");   // lower arm much
  bridge.tx("regbot madd vel=0.05:dist=0.2\n");
  bridge.tx("regbot madd vel=0.0:time=0.5\n");
  bridge.tx("regbot madd servo=1, pservo=300, vservo=100: time=10.0\n");   // raise arm
  bridge.tx("regbot madd vel=0.05:dist=0.1\n");

  bridge.tx("regbot madd vel=-0.1:time=1.5\n");
  bridge.tx("regbot madd vel=0.2: lv=20, dist=0.05\n");
  bridge.tx("regbot madd vel=0.2, edgel=0, white=1: dist=0.18\n");
  bridge.tx("regbot madd vel=0.0:time=0.5\n");

// Fourth step:
  bridge.tx("regbot madd servo=1, pservo=-450, vservo=120: time=6.0\n");   // lower arm much
  bridge.tx("regbot madd vel=0.05:dist=0.2\n");
  bridge.tx("regbot madd vel=0.0:time=0.5\n");
  bridge.tx("regbot madd servo=1, pservo=300, vservo=100: time=10.0\n");   // raise arm
  bridge.tx("regbot madd vel=0.05:dist=0.1\n");

  bridge.tx("regbot madd vel=-0.1:time=1.5\n");
  bridge.tx("regbot madd vel=0.2: lv=20, dist=0.05\n");
  bridge.tx("regbot madd vel=0.2, edgel=0, white=1: dist=0.18\n");
  bridge.tx("regbot madd vel=0.0:time=0.5\n");

// Fifth step:
  bridge.tx("regbot madd servo=1, pservo=-450, vservo=120: time=6.0\n");   // lower arm much
  bridge.tx("regbot madd vel=0.05:dist=0.2\n");
  bridge.tx("regbot madd vel=0.0:time=0.5\n");
  bridge.tx("regbot madd servo=1, pservo=300, vservo=100: time=10.0\n");   // raise arm
  bridge.tx("regbot madd vel=0.05:dist=0.1\n");

// Go home
  bridge.tx("regbot madd vel=-0.1:time=1.5\n");
  bridge.tx("regbot madd vel=0.2: lv=20, dist=0.05\n"); 
  bridge.tx("regbot madd vel=0.3, edgel=0, white=1:xl>15\n");
  bridge.tx("regbot madd vel=0.2, tr=0.1: turn=-90 \n"); 
  bridge.tx("regbot madd vel=0.0: time=0.1\n"); 
  bridge.tx("regbot madd vel=0.2: dist=0.2\n"); 
  bridge.tx("regbot madd vel=0.2: lv=20, dist=0.1\n"); 
  bridge.tx("regbot madd vel=0.2, edgel=0.0, white=1: dist=0.3\n"); 
  bridge.tx("regbot madd vel=0.0: time=0.1\n"); 
  bridge.tx("regbot madd vel=0.2, tr=0.0: turn=180 \n");

  bridge.tx("regbot start \n");

  event.waitForEvent(0);
	cout << "user:# Waiting  for " + string(__func__) + " to finish" << endl;

}

void offHeadingController()
{
  // // reset bridge mission
  // bridge.tx("regbot mclear\n");
  // event.clearEvents();
  // usleep(2000);

  // bridge.tx("regbot:ctrn subscribe -1");
  bridge.tx("regbot ctrn 0 1 0 1 9999.99 1 0 1 1 0 1 1 0 1 1 0 1 9999.99 1 0 1 0 1 1 0 9999.99\n");

  // bridge.tx("regbot start\n");
  // event.waitForEvent(0);
}

void onHeadingController()
{
  // reset bridge mission
  bridge.tx("regbot mclear\n");
  event.clearEvents();
  usleep(2000);

  // bridge.tx("regbot:ctrn subscribe -1");
  bridge.tx("regbot ctrn 1 1 1 1 9999.99 1 1 1 1 1 1 1 1 1 1 0 1 9999.99 1 0 1 0 1 1 1 9999.99\n");

  bridge.tx("regbot start\n");
  event.waitForEvent(0);
}

void tunnel()
{
  bridge.tx("regbot mclear\n");
  event.clearEvents();
  usleep(200);

  // find home1 and u-turn
  bridge.tx("regbot madd servo=1, pservo=900, vservo=120: time=1.0\n");   // raise arm 
  bridge.tx("regbot madd vel=0.2: dist=0.05\n");
  bridge.tx("regbot madd vel=0.2: lv=20, dist=0.8\n");
  bridge.tx("regbot madd vel=0.2, edger=0, white=1: dist=1.0, lv<4, xl>15\n");
  bridge.tx("regbot madd vel=0.2, tr=0.0: turn=180\n");
  
  // follow white line, past cross line
  bridge.tx("regbot madd vel=0.3: dist=0.05\n");
  bridge.tx("regbot madd vel=0.3: lv=20, dist=0.8\n");

  // find cross line
  bridge.tx("regbot madd vel=0.3, edger=0, white=1: dist=1.5, lv<4, xl>15\n");
  // move past XL
  bridge.tx("regbot madd vel=0.3, edger=0, white=1: dist=0.3, lv<4\n");
  
  // turn left towards tunnel
  bridge.tx("regbot madd vel=0.2, tr=0.3: turn=90\n");
  bridge.tx("regbot madd vel=0.3: ir2<0.15\n");
  bridge.tx("regbot madd vel=0.0: time=1.0\n");

  // reset bridge mission
  bridge.tx("regbot start\n");
  event.waitForEvent(0);
  bridge.tx("regbot mclear\n");
  event.clearEvents();
  usleep(20);

  bridge.tx("regbot madd vel=0.2, tr=0.13:turn=-90\n");
  bridge.tx("regbot madd vel=0.0: time=1.0\n");

  // go to front door and push open
  // bridge.tx("regbot madd vel=0.2: ir2<0.1\n");   // detect front door
  bridge.tx("regbot madd vel=0.2: dist=0.1\n");
  bridge.tx("regbot madd vel=0.0: time=1.0\n");
  bridge.tx("regbot madd vel=0.6: dist=0.15\n");    // knock front door
 
  // reset bridge mission
  bridge.tx("regbot start\n");
  event.waitForEvent(0);
  bridge.tx("regbot mclear\n");
  event.clearEvents();
  usleep(20);

  // turn left twice into tunnel
  bridge.tx("regbot madd vel=0.3, tr=0.35: turn=90\n");
  bridge.tx("regbot madd vel=0.0: time=1.5\n");

  // reset bridge mission
  bridge.tx("regbot start\n");
  event.waitForEvent(0);
  bridge.tx("regbot mclear\n");
  event.clearEvents();
  usleep(20);

  bridge.tx("regbot madd vel=0.3, tr=0.12: turn=93\n");
  bridge.tx("regbot madd vel=0.0: time=1.0\n");

  // reset bridge mission
  bridge.tx("regbot start\n");
  event.waitForEvent(0);
  bridge.tx("regbot mclear\n");
  event.clearEvents();
  usleep(20);

  // go to back door and push open
  bridge.tx("regbot madd vel=0.4: ir2<0.15\n");
  bridge.tx("regbot madd vel=0.4: dist=0.4\n");


  // reset bridge mission
  bridge.tx("regbot start\n");
  event.waitForEvent(0);
  bridge.tx("regbot mclear\n");
  event.clearEvents();
  usleep(20);

  // 1st turn right and go straight
  bridge.tx("regbot madd vel=0.4, tr=0.34: turn=-90\n");
  bridge.tx("regbot madd vel=0.0: time=0.8\n");
  bridge.tx("regbot madd vel=0.4: dist=0.1\n");

  // reset bridge mission
  bridge.tx("regbot start\n");
  event.waitForEvent(0);
  bridge.tx("regbot mclear\n");
  event.clearEvents();
  usleep(20);

  // 2nd turn right and go straight (along left side)
  bridge.tx("regbot madd vel=0.4, tr=0.28: turn=-90\n");
  bridge.tx("regbot madd vel=0.0: time=0.8\n");
  bridge.tx("regbot madd vel=0.4: dist=0.67\n");

  // reset bridge mission
  bridge.tx("regbot start\n");
  event.waitForEvent(0);
  bridge.tx("regbot mclear\n");
  event.clearEvents();
  usleep(20);

  // 3rd turn right and go straight, close front door
  bridge.tx("regbot madd vel=0.4: dist=0.1\n");
  bridge.tx("regbot madd vel=0.4, tr=0.3: turn=-130\n");
  bridge.tx("regbot madd vel=0.2: dist=0.20 \n");
  bridge.tx("regbot madd vel=-0.2: dist=0.20 \n");

  // reset bridge mission
  bridge.tx("regbot start\n");
  event.waitForEvent(0);
  bridge.tx("regbot mclear\n");
  event.clearEvents();
  usleep(20);

  bridge.tx("regbot madd vel=0.4, tr=0.3: turn=45\n");
  bridge.tx("regbot madd vel=0.4: dist=0.5\n");
  

  // bridge.tx("regbot madd vel=0.4, tr=0.3: turn=-90\n");
  // bridge.tx("regbot madd vel=0.0: time=0.8\n");
  // bridge.tx("regbot madd vel=0.4: dist=0.9\n");

  // reset bridge mission
  bridge.tx("regbot start\n");
  event.waitForEvent(0);
  bridge.tx("regbot mclear\n");
  event.clearEvents();
  usleep(20);

  // 4th turn right and go straight (along right side)
  bridge.tx("regbot madd vel=0.4, tr=0.3: turn=-110\n");
  bridge.tx("regbot madd vel=0.0: time=0.8\n");
  bridge.tx("regbot madd vel=0.45: dist=0.65\n");    // knocks back door

  // reset bridge mission
  bridge.tx("regbot start\n");
  event.waitForEvent(0);
  bridge.tx("regbot mclear\n");
  event.clearEvents();
  usleep(20);

  // 5th turn right and go straight, close back door
  bridge.tx("regbot madd vel=0.4, tr=0.3: turn=-90\n");
  bridge.tx("regbot madd vel=0.0: time=0.8\n");
  bridge.tx("regbot madd vel=0.3: dist=0.34\n");     // push against back door
  bridge.tx("regbot madd vel=0.0: time=0.5\n");
  
  // start this mission
  bridge.tx("regbot start\n");
  cout << "user:# Waiting for " + string(__func__) + " to finish\n" << endl;
  event.waitForEvent(0);
  cout <<  "user:# obstacle " + string(__func__) + " has finished\n" << endl;
}

void tunnelToHome()
{
  bridge.tx("regbot mclear\n");
  event.clearEvents();
  usleep(2000);
  
  // reverse and return to line
  bridge.tx("regbot madd vel=-0.4: lv=20, dist=0.5\n");
  bridge.tx("regbot madd vel=-0.4, tr=0.5: turn=170\n");
  bridge.tx("regbot madd vel=0.0: time=0.8\n");
  bridge.tx("regbot madd vel=0.3: dist=0.5\n");
  bridge.tx("regbot madd vel=0.3: lv=20, dist=2.0\n");

  bridge.tx("regbot madd vel=0.2, edger=0, white=1: dist=0.2, lv<4\n");
  bridge.tx("regbot madd vel=0.3, edger=0, white=1: xl>15, lv<4\n");
  bridge.tx("regbot madd vel=0.0: time=0.5\n");  
  bridge.tx("regbot madd vel=-0.2: dist=0.2\n");  
  bridge.tx("regbot madd vel=0.0: time=0.5\n");  

  // start this mission
  bridge.tx("regbot start\n");
  cout << "user:# Waiting for " + string(__func__) + " to finish" << endl;
  event.waitForEvent(0);
  cout << "user:# obstacle " + string(__func__) + " has finished" << endl;
}

void axe()
{
  bridge.tx("regbot mclear\n");
  event.clearEvents();
  usleep(2000);
  
  // initialise gripper position
  bridge.tx("regbot madd servo=1, pservo=700, vservo=200: time=5.0\n");   // Raise arm
  bridge.tx("regbot madd servo=2, pservo=100: time=4.0\n");   // Open gripper

  // find cross line and turn
  bridge.tx("regbot madd vel=0.2: lv=20, dist=0.8\n");
  bridge.tx("regbot madd vel=0.2, edger=0.0, white=1: dist=1.0, xl>16  \n");
  bridge.tx("regbot madd vel=0.2, tr=0.1: turn=-85 \n");
  bridge.tx("regbot madd vel=0.0: time=1\n");

  // travel towards axe
  bridge.tx("regbot madd vel=0.2, edger=0.0, white=1: dist=0.3, lv<4\n");
  bridge.tx("regbot madd vel=0.2: dist=0.5\n");
  bridge.tx("regbot madd vel=0.2, edger=0.0, white=1: dist=0.4, lv<4\n");

  // wait for opening
  bridge.tx("regbot madd vel=0.0: time=0.25  \n");
  bridge.tx("regbot madd :ir2>0.4 \n");
  bridge.tx("regbot madd :ir2<0.3 \n");
  bridge.tx("regbot madd :ir2>0.4 \n");

  // speed through opening
  bridge.tx("regbot madd vel=0.1, edger=0.0, white=1: dist=0.1, lv<4  \n");
  bridge.tx("regbot madd vel=0.7: dist=0.7 \n");
  
  // start this mission
  bridge.tx("regbot start\n");
  cout << "user:# Waiting for" + string(__func__) + "to finish\n" << endl;
  event.waitForEvent(0);
  cout <<  "user:# obstacle " + string(__func__) + " has finished\n" << endl;
}

void fastTrack()
{
  bridge.tx("regbot mclear\n");
  event.clearEvents();
  usleep(2000);

  // find white line and turn
  bridge.tx("regbot madd vel=0.2: lv=20, dist=1.5\n");
  bridge.tx("regbot madd vel=0.2, tr=0.1: turn=-85 \n");
  bridge.tx("regbot madd vel=0.0: time=0.5 \n");

  // start zooming down track
  bridge.tx("regbot madd vel=0.4: dist=0.5, lv=20\n");
  bridge.tx("regbot madd vel=0.4, edger=0.0, white=1:dist=0.4, lv<4\n");
  bridge.tx("regbot madd vel=1.0, edger=0.0, white=1:dist=12, lv<4\n");
  bridge.tx("regbot madd vel=0.0: time=0.5\n");

  bridge.tx("regbot madd vel=0.0: time=0.5\n");

  // start this mission
  bridge.tx("regbot start\n");
  cout << "Waiting for " + string(__func__) + "to finish" << endl;
  event.waitForEvent(0);
  cout << "user:# obstacle " + string(__func__) + " has finished" << endl;
}

void fastTrackToHome()
{
  bridge.tx("regbot mclear\n");
  event.clearEvents();
  usleep(2000);
  
  // turn right after finishing
  bridge.tx("regbot madd vel=0.3, tr=0.1: turn=-90 \n");
  bridge.tx("regbot madd vel=0.0: time=0.5 \n");
  bridge.tx("regbot madd vel=0.3: dist=2.0, lv=20\n");

  // return to home
  bridge.tx("regbot madd vel=0.3, tr=0.1: turn=85 \n");
  bridge.tx("regbot madd vel=0.0: time=0.5 \n");
  bridge.tx("regbot madd vel=0.3: dist=0.5, lv=20\n");
  bridge.tx("regbot madd vel=0.4, edger=0.0, white=1: dist=1.2\n");

  // start this mission
  bridge.tx("regbot start\n");
  cout << "user:# Waiting for " + string(__func__) + " to finish" << endl;
  event.waitForEvent(0);
  cout << "user:# obstacle " + string(__func__) + " has finished" << endl;
}

void roundaboutGate()
{
  bridge.tx("regbot mclear\n");
  event.clearEvents();
  usleep(2000);
  bridge.tx("regbot madd vel=0.0: time=0.1  \n");
  bridge.tx("regbot madd servo=1,pservo=400,vservo=100:time=5.0\n"); // Arm a low position to make robot front heavy
  bridge.tx("regbot madd servo=2,pservo=-500:time=2.0\n");
  bridge.tx("regbot madd vel=0.3, edgel=0.0, white=1: dist=0.84 \n");
  bridge.tx("regbot madd vel=0.2: dist=0.3 \n");
  bridge.tx("regbot madd vel=0.2, edger=0.0, white=1: dist=3.0, xl>15 \n");
  bridge.tx("regbot madd vel=0.0: time=0.2  \n");
  bridge.tx("regbot madd vel=0.1, tr=0.001: turn=70 \n");
  bridge.tx("regbot madd vel=0.0: time=0.1  \n");
  bridge.tx("regbot madd : ir2<0.4  \n");
  bridge.tx("regbot madd : ir2>0.4 \n");

// Find white line and go a bit
  bridge.tx("regbot madd vel=0.3, tr=3: dist=0.3, turn=180\n"); 
  bridge.tx("regbot madd vel=0.2, edgel=0.0, white=1: dist=1.0\n"); 
  bridge.tx("regbot madd vel=0.0: time=0.1\n");

  // Turn and back up
  bridge.tx("regbot madd vel=0.0: time=0.1  \n");
  bridge.tx("regbot madd vel=0.2, tr=0.0: turn=95 \n");
  bridge.tx("regbot madd vel=0.0: time=0.1  \n");
  bridge.tx("regbot madd vel=-0.2: dist=-0.8  \n");
  bridge.tx("regbot madd vel=0.0: time=0.1  \n");

//   // Go around in circle
  bridge.tx("regbot madd servo=1,pservo=925,vservo=100:time=6.0\n"); // Raise arm to avoid gates
  bridge.tx("regbot madd vel=0.1: dist=0.15  \n");
  bridge.tx("regbot madd vel=0.2, tr=0.0: turn=110 \n");
  bridge.tx("regbot madd vel=0.1: dist=0.1\n");
  bridge.tx("regbot madd vel=0.0: time=0.1  \n");
  bridge.tx("regbot madd vel=0.2, tr=0.4: dist=1.9, turn=500 \n");
  bridge.tx("regbot madd vel=0.0: time=0.1 \n");



  // start this mission
  bridge.tx("regbot start\n");
  // wait until finished
  cout << "user:# Waiting for " + string(__func__) + " to finish\n" << endl;
  event.waitForEvent(0);
  cout <<  "user:# obstacle " + string(__func__) + " has finished\n" << endl;
}

void roundaboutHome()
{
  bridge.tx("regbot mclear\n");
  event.clearEvents();
  usleep(2000); 

  bridge.tx("regbot madd vel=0.2: dist=0.1\n");
  bridge.tx("regbot madd vel=0.1, tr=0.0: turn=-60 \n");
  bridge.tx("regbot madd vel=0.0: time=0.1\n"); 
  bridge.tx("regbot madd : ir2<0.4 \n");
  bridge.tx("regbot madd : ir2>0.4 \n");
  bridge.tx("regbot madd vel=0.0: time=0.1\n"); 
  bridge.tx("regbot madd vel=0.3: dist=3.0, lv=20\n");
  bridge.tx("regbot madd vel=0.1, tr=0.1: turn=100 \n");
  bridge.tx("regbot madd vel=0.2, edger=0.0, white=1: dist=0.2\n"); 
  bridge.tx("regbot madd vel=0.2: dist=2.0, xl>16 \n"); 
  bridge.tx("regbot madd vel=0.0: time=0.1\n"); 
  bridge.tx("regbot madd vel=0.2, tr=0.05: turn=-90 \n"); 
  bridge.tx("regbot madd vel=0.0: time=0.1\n"); 
  bridge.tx("regbot madd vel=0.2, edger=0.0, white=1: dist=2.3\n"); 
  bridge.tx("regbot madd vel=0.0: time=0.1\n"); 
  bridge.tx("regbot madd vel=0.2, tr=0.0: turn=180 \n");

  // start this mission
  bridge.tx("regbot start\n");
  cout << "user:# Waiting for " + string(__func__) + " to finish\n" << endl;
  event.waitForEvent(0);
  cout <<  "user:# obstacle " + string(__func__) + " has finished\n" << endl;
}

void goal()
{
  bridge.tx("regbot mclear\n");
  event.clearEvents();
  
  bridge.tx("regbot madd servo=1, pservo=600, vservo=100: time=3.0\n");  // raise arm
  bridge.tx("regbot madd servo=2, pservo=-200, vservo=120: time=3.0\n");  // close gripper

  // find home1 and u-turn
  bridge.tx("regbot madd vel=0.2: dist=0.05\n");
  bridge.tx("regbot madd vel=0.2: lv=20, dist=0.8\n");
  bridge.tx("regbot madd vel=0.2, edger=0, white=1: dist=1.0, lv<4, xl>15\n");
  bridge.tx("regbot madd vel=0.2, tr=0.0: turn=180\n");

  // travel towards goal
  bridge.tx("regbot madd vel=0.3: dist=0.05\n");
  bridge.tx("regbot madd vel=0.3: lv=20, dist=0.8\n");
  bridge.tx("regbot madd vel=0.3, edgel=0, white=1: lv<4, ir2<0.4\n");
  bridge.tx("regbot madd vel=0.0: time=0.5\n");
  bridge.tx("regbot madd servo=1, pservo=200, vservo=120: time=3.0\n");  // lower arm
  bridge.tx("regbot madd vel=0.3: dist=0.2, lv<4\n");
  bridge.tx("regbot madd vel=0.0: time=0.5\n");

  // start this mission
  bridge.tx("regbot start\n");
  cout << "user:# Waiting for " + string(__func__) + " to finish\n" << endl;
  event.waitForEvent(0);
  cout <<  "user:# obstacle " + string(__func__) + " has finished\n" << endl;
}

void deliverBallFromStartRampNoHead()
{
  bridge.tx("regbot mclear\n");
  event.clearEvents();
  usleep(2000);
  bridge.tx("regbot madd servo=2,pservo=-600:time=1.0 \n"); 
  bridge.tx("regbot madd servo=1, pservo=200,vservo=120:time=1.0\n");
  bridge.tx("regbot madd vel=0.2:dist=0.1\n");
  bridge.tx("regbot madd vel=0.2: lv=20, dist=0.05\n"); 
  bridge.tx("regbot madd vel=0.2, edgel=0, white=1:lv<4, xl>15\n");
  bridge.tx("regbot madd vel=0.2: dist=0.1\n");
  bridge.tx("regbot madd vel=0.2: lv=20, dist=0.05\n"); 
  bridge.tx("regbot madd vel=0.2, edger=0, white=1: dist=1.0\n"); 
  bridge.tx("regbot madd vel=0.2:dist=0.3\n"); 
  bridge.tx("regbot madd vel=0.0:time=2.0\n");
  bridge.tx("regbot madd vel=0.05,tr=0.0:turn=-18\n");
  bridge.tx("regbot madd vel=0.05:dist=0.17\n");
  bridge.tx("regbot madd vel=0.0:time=1.0\n");
  bridge.tx("regbot madd servo=1,pservo=0,vservo=100:time=6.0\n");
  bridge.tx("regbot madd vel=0.0: time=0.1 \n");
  bridge.tx("regbot madd servo=2,pservo=-350:time=2.0 \n");
  bridge.tx("regbot madd vel=0.0:time=1.0\n");
  bridge.tx("regbot madd vel=0.05,tr=0.0:turn=-30\n");

  bridge.tx("regbot madd vel=0.0:time=0.5\n");

  // Return to home
  bridge.tx("regbot madd servo=1, pservo=200,vservo=200:time=3.0\n");
  bridge.tx("regbot madd vel=0.0:time=2.0\n");

  bridge.tx("regbot madd vel=0.1,tr=0.0:turn=120\n");

  bridge.tx("regbot madd vel=0.3: dist=0.5, lv=20\n");
  bridge.tx("regbot madd vel=0.3, edger=0.0, white=1: dist=3\n"); // Drive till end of ramp
  bridge.tx("regbot madd vel=0.2, edger=0.0, white=1: ir2<0.4\n");
  bridge.tx("regbot madd vel=0.2, tr=0.0: turn=180\n");
  bridge.tx("regbot madd vel=0.2, edger=0, white=1: lv<4, dist=1.1\n");

  bridge.tx("regbot start \n");

  event.waitForEvent(0);
	cout << "user:# Waiting  for " + string(__func__) + " to finish" << endl;
}

void deliverBallFromStartRamp()
{
  bridge.tx("regbot mclear\n");
  event.clearEvents();
  usleep(2000);   
  bridge.tx("regbot madd servo=2, pservo=-600: time=1.0 \n"); 
  bridge.tx("regbot madd servo=1, pservo=200, vservo=120: time=1.0\n");
  bridge.tx("regbot madd vel=0.2:dist=0.1\n");
  bridge.tx("regbot madd vel=0.2: lv=20, dist=0.05\n"); 
  bridge.tx("regbot madd vel=0.2, edgel=0, white=1:lv<4, xl>15\n");
  bridge.tx("regbot madd vel=0.2: dist=0.1\n");
  bridge.tx("regbot madd vel=0.2: lv=20, dist=0.05\n"); 
  bridge.tx("regbot madd vel=0.2, edger=0, white=1: dist=1.0\n"); 
  bridge.tx("regbot madd vel=0.2:dist=0.3\n"); 
  bridge.tx("regbot madd vel=0.0:time=2.0\n");
  bridge.tx("regbot madd vel=0.1,tr=0.0:turn=-20\n");   // FIX: Needs fine tuning
  bridge.tx("regbot madd vel=0.1:dist=0.25\n");     // FIX: Need fine tuning
  bridge.tx("regbot madd vel=0.0:time=1.0\n");
  bridge.tx("regbot madd servo=1,pservo=0,vservo=100:time=6.0\n");
  bridge.tx("regbot madd servo=2,pservo=-150:time=2.0 \n");
  bridge.tx("regbot madd vel=0.0:time=1.0\n");
  bridge.tx("regbot madd vel=0.1,tr=0.0:turn=-35\n");

  bridge.tx("regbot madd vel=0.0:time=0.5\n");

  // Return to home
  bridge.tx("regbot madd servo=1, pservo=200,vservo=200:time=3.0\n");
  bridge.tx("regbot madd vel=0.0:time=2.0\n");

  bridge.tx("regbot madd vel=0.1,tr=0.0:turn=120\n");

  bridge.tx("regbot madd vel=0.3: dist=0.5, lv=20\n");
  bridge.tx("regbot madd vel=0.3, edger=0.0, white=1: dist=3\n"); // Drive till end of ramp
  bridge.tx("regbot madd vel=0.2, edger=0.0, white=1: ir2<0.4\n");
  bridge.tx("regbot madd vel=0.2, tr=0.0: turn=180\n");
  bridge.tx("regbot madd vel=0.2, edger=0, white=1: lv<4, dist=1.1\n");

  bridge.tx("regbot start \n");

  event.waitForEvent(0);
	cout << "user:# Waiting  for " + string(__func__) + " to finish" << endl;
}

void armNeutralPos()
{
  bridge.tx("regbot mclear\n");
  event.clearEvents();
  usleep(2000);

  bridge.tx("regbot madd servo=2,pservo=100:time=2\n"); 
  bridge.tx("regbot madd servo=1,pservo=700,vservo=100:time=8\n");  

  bridge.tx("regbot start \n");

  event.waitForEvent(0);
	cout << "user:# Waiting  for " + string(__func__) + " to finish" << endl;
}

void gripExample()
{
  bridge.tx("regbot mclear\n");
  event.clearEvents();
  usleep(2000);

  // Neutral
  bridge.tx("regbot madd servo=2,pservo=100:time=2\n"); 
  bridge.tx("regbot madd servo=1,pservo=600,vservo=100:time=8\n"); 

  // Grip ball
  bridge.tx("regbot madd servo=1, pservo=-100,vservo=100:time=10 \n");
  bridge.tx("regbot madd servo=2,pservo=-400:time=2 \n");

  // Lift a little
  bridge.tx("regbot madd servo=1,pservo=100,vservo=100:time=5\n");

  // Open
  bridge.tx("regbot madd servo=1,pservo=-100,vservo=100:time=5\n");
  bridge.tx("regbot madd servo=2,pservo=-200:time=2 \n");

  // start this mission
  bridge.tx("regbot start\n");
  cout << "user:# Waiting for " + string(__func__) + " to finish" << endl;
  event.waitForEvent(0);
  cout << "user:# obstacle " + string(__func__) + " has finished" << endl;
}

void patrolSequence()
{
  
	bridge.tx("regbot mclear\n");
	event.clearEvents();
	usleep(2000);
	bridge.tx("regbot madd vel=0.0: time=0.1 \n");
	bridge.tx("regbot madd vel=0.3, edger=0.0, white=1: dist=1.5 \n");
	bridge.tx("regbot madd vel=0.0: time:0.1 \n");

	//start this mission
	cout << "user:# Waiting  for " + string(__func__) + " to finish" << endl;
        bridge.tx("regbot start \n");
	while (not event.gotEvent(0))
  	{
  		std::cout << "encoder1:" << enc.e3<< endl;
		  std::cout << "encoder2:" << enc.e4 << endl;
		  usleep(10000);
      if (enc.e4 > 1000)
      {
        bridge.tx("regbot event=0\n");
	      event.clearEvents();
	      usleep(2000);
        bridge.tx("regbot madd vel=0.0: time=0.1 \n");
        bridge.tx("regbot start \n");
        break;
      }
  	}
	event.waitForEvent(0);
	cout << "user:# Waiting  for " + string(__func__) + " to finish" << endl;
}

void guillotineRampSequence()
{
  guillotine();
  ramp();
}

void tunnelSequence()
{
  // offHeadingController();
  tunnel();
  tunnelToHome();
}

void axeFastTrackSequence()
{
  axe();
  fastTrack();
  fastTrackToHome();
}

void getBallOnRamp()
{
  ballOnRamp();  
  deliverBallFromStartRamp();
}

void seesawST()
{
  bridge.tx("regbot mclear\n");
  event.clearEvents();

  bridge.tx("regbot madd servo=1, pservo=600, vservo=200: time=5.0\n");   // Raise arm
  bridge.tx("regbot madd servo=2, pservo=325: time=4.0\n");   // Open gripper

  // follow white line, find cross line
  bridge.tx("regbot madd vel=0.3: dist=0.1\n");
  bridge.tx("regbot madd vel=0.3: lv=20, dist=0.5\n");
  bridge.tx("regbot madd vel=0.3, edgel=0, white=1: lv<4, xl>15\n");

  // turn into seeseaw
  bridge.tx("regbot madd vel=0.2: dist=0.11\n");  // Drive a bit forward
  bridge.tx("regbot madd vel=0.05, tr=0.0: turn=90, time=10.0\n"); // Turn left
  bridge.tx("regbot madd vel=0.0: time=0.1\n");
  bridge.tx("regbot madd vel=-0.1: dist=-0.1\n");  // Go a bit backwards in order to have more line to find

  // Handle transition between ramp and seasaw
  bridge.tx("regbot madd vel=0.1: dist=0.1\n");  // 
  bridge.tx("regbot madd vel=0.0: time=0.1\n");  // 
  bridge.tx("regbot madd servo=1, pservo=0, vservo=120: time=5.0\n");   // Lower arm
  bridge.tx("regbot madd vel=0.0: time=0.1\n");
  bridge.tx("regbot madd vel=0.1: lv=20, dist=0.5\n");  // 

  bridge.tx("regbot madd vel=0.0: time=0.1\n");
  bridge.tx("regbot madd vel=0.1, edgel=0, white=1: dist=0.3\n");  //
  bridge.tx("regbot madd vel=0.1, edger=1.0, white=1: dist=0.875\n");
  bridge.tx("regbot madd vel=0.0: time=0.1\n");
  bridge.tx("regbot madd servo=1, pservo=0, vservo=120: time=5.0\n");   // Lower arm

  bridge.tx("regbot madd vel=0.0: time=0.1\n");  // 
  bridge.tx("regbot madd servo=1, pservo=-75, vservo=120: time=2.0\n");   // Lower arm

 // Handle down the seasaw
  bridge.tx("regbot madd servo=2, pservo=-600: time=4.0\n");   // Close gripper
  bridge.tx("regbot madd servo=1, pservo=0, vservo=120: time=2.0\n");   // Lift arm so gripper can close properly
  bridge.tx("regbot madd servo=1, pservo=-400, vservo=100: time=6.0\n");   // lower arm much
  bridge.tx("regbot madd vel=0.0: time=0.1\n");
  bridge.tx("regbot madd vel=0.05 tr=3.0: turn=180 dist=0.8\n");
  bridge.tx("regbot madd vel=0.0: time=0.1\n");
  bridge.tx("regbot madd servo=1, pservo=400, vservo=120: time=9.0\n");   // Lift 
  bridge.tx("regbot madd vel=0.0: time=0.1\n");
  bridge.tx("regbot madd vel=0.1: lv=20, dist=0.5\n");  // 
  bridge.tx("regbot madd vel=0.1, edger=0, white=1: lv<4, dist=0.5\n");  //

 // find line after seasaw and home
  bridge.tx("regbot madd vel=0.0: time=0.1\n"); 
  bridge.tx("regbot madd vel=0.2: dist=2.0, xl>16 \n"); 
  bridge.tx("regbot madd vel=0.0: time=0.1\n"); 
  bridge.tx("regbot madd vel=0.2, tr=0.05: turn=-90 \n"); 
  bridge.tx("regbot madd vel=0.0: time=0.1\n"); 
  bridge.tx("regbot madd vel=0.2, edger=0.0, white=1: dist=1.5\n"); 
  bridge.tx("regbot madd vel=0.0: time=0.1\n"); 
  bridge.tx("regbot madd vel=0.2, tr=0.0: turn=180 \n");
  
  // start this mission
  bridge.tx("regbot start\n");
  event.waitForEvent(0);

}

void aruco_mission(){
  bool aruco = vision.aruco_mission(560);
}

int main(int argc, char **argv)
{
  cout << "# Hello, Robobot user mission starting ..." << endl;
  setup(argc, argv);
  cout << "After setup" << endl;

// ALL THIS WORKS AND IS TESTED //
  guillotineRampSequence();
  axeFastTrackSequence();
  roundaboutGate();
  roundaboutHome();

  // tunnelSequence();
  // restartRamp();
  // stairs();

  // restartRamp();
  // getBallOnRamp();

  restartRamp();
  seesawST();

  // restartRamp();
  // deliverBallFromStartRampNoHead();

  aruco_mission();

  // restartRamp();
  // ballOnRamp();

  // restartRamp();  
  // seesawST();          // WITHOUT heading controller
  // restartRamp();
  // deliverBallFromStartRamp();
  
  goal();

// TILL HERE //

// WORKS AND TESTED INDIVIDUALLY //
  
 
// NOT TESTED
  // armNeutralPos();
  // patrolSequence();           // Stopping robot does NOT work. Following(stop/go) is NOT implemented

// Exit mission
  cout << "# Robobot user mission finished ..." << endl;
  vision.stop();      // remember to close camera
  sound.say("Mission complete", 0.2);
  while (sound.isSaying())
    sleep(1);
  bridge.tx("regbot mute 1\n");
  return 0;
}