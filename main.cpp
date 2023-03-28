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

void moveTest()
{
  bridge.tx("regbot mclear\n");
  event.clearEvents();
  bridge.tx("regbot madd vel=0.2: dist=0.3\n");
  bridge.tx("regbot start\n");
  event.waitForEvent(0);
}



void gripExample()
{
  bridge.tx("regbot mclear\n");
  event.clearEvents();

bridge.tx("regbot madd servo=2, pservo=100: time=2 \n");
bridge.tx("regbot madd servo=1, pservo=-350, vservo=100: time=2 \n");
bridge.tx("regbot madd vel=-0.3: dist=-1 \n");
bridge.tx("regbot madd vel=0: time=0.5 \n");
bridge.tx("regbot madd vel=0.3: dist=1 \n");
bridge.tx("regbot madd vel=0: time=0.5\n");
bridge.tx("regbot madd servo=1, pservo=-550,vservo=100:time=5 \n");
bridge.tx("regbot madd servo=2,pservo=-600:time=2 \n");
bridge.tx("regbot madd servo=1,pservo=-350,vservo=100:time=5 \n");
bridge.tx("regbot madd vel=0.3:dist=1 \n");
bridge.tx("regbot madd vel=0:time=0.5 \n");
bridge.tx("regbot madd servo=1,pservo=-550,vservo=100:time=5 \n");
bridge.tx("regbot madd servo=2,pservo=100:time=2 \n");
bridge.tx("regbot madd servo=1,pservo=-350,vservo=100:time=5 \n");
bridge.tx("regbot madd vel=-0.3:dist=-2 \n");
bridge.tx("regbot madd vel=0: time=0.5 \n");

  // start this mission
  bridge.tx("regbot start\n");
  cout << "user:# Waiting for " + string(__func__) + " to finish" << endl;
  event.waitForEvent(0);
  cout << "user:# obstacle " + string(__func__) + " has finished" << endl;
}

void testMethod()
{
  bridge.tx("regbot mclear\n");
  event.clearEvents();

  // leave line, find line
  bridge.tx("regbot madd vel=0.2: dist=0.7\n");
  bridge.tx("regbot madd vel=0.2: lv=20\n");
  bridge.tx("regbot madd vel=0.3,tr=0.05: turn=90\n");
  bridge.tx("regbot madd vel=0.2, edger=0, white=1: lv<4, xl>15\n");
  bridge.tx("regbot madd vel=0.0: time=0\n");

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
  usleep(2000);

  bridge.tx("regbot madd servo=1, pservo=700, vservo=120: time=4\n");   //set arm to top position
  bridge.tx("regbot madd vel=0.3: dist=0.3\n");
  bridge.tx("regbot madd vel=0.5: dist=0.5, lv=20\n");
  bridge.tx("regbot madd vel=0.6, edger=0.0, white=1: dist=3, lv<3\n"); // ends slightly after y-junction

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

  bridge.tx("regbot madd vel=0.6: dist=0.5, lv=20\n");
  
  bridge.tx("regbot madd vel=0.6, edger=0.0, white=1: dist=11.0, lv<3\n");
  bridge.tx("regbot madd vel=0.0: time=0\n");
  // complete the downRamp
  bridge.tx("regbot madd vel=0.6, edger=0.0, white=1: dist=1.8, lv<3\n");

  // leave line, find line
  bridge.tx("regbot madd vel=0.2: dist=0.45\n");
  bridge.tx("regbot madd vel=0.2: lv=20\n");
  bridge.tx("regbot madd vel=0.3,tr=0.05: turn=90\n");
  bridge.tx("regbot madd vel=0.2, edger=0, white=1: lv<4, xl>15\n");
  bridge.tx("regbot madd vel=0.0: time=0\n");

  // start this mission
  bridge.tx("regbot start\n");
  cout << "user:# Waiting for " + string(__func__) + " to finish" << endl;
  event.waitForEvent(0);
  cout << "user:# obstacle " + string(__func__) + " has finished" << endl;
}

void restartRamp()
{
  bridge.tx("regbot mclear\n");
  event.clearEvents();

  // follow lines to restart ramp
  bridge.tx("regbot madd vel=0.3: dist=0.5, lv=20\n");
  bridge.tx("regbot madd vel=0.4, edger=0, white=1: lv<4\n");

  // go past line break
  bridge.tx("regbot madd vel=0.4: dist=0.3\n");

  // detect 1st cross line and go past it
  bridge.tx("regbot madd vel=0.5, edger=0, white=1: lv<4, xl>15\n");
  bridge.tx("regbot madd vel=0.5: dist=0.2\n");

  // detect 2nd cross line and go past it
  bridge.tx("regbot madd vel=0.5, edger=0, white=1: lv<4, xl>15\n");
  bridge.tx("regbot madd vel=0.5: dist=0.2\n");

  // follow line for another 0.5m
  bridge.tx("regbot madd vel=0.5, edger=0, white=1: lv<4, dist=0.5\n");
  // go straight to exit line track, go straight until line detected
  bridge.tx("regbot madd vel=0.5: dist=0.8\n");
  bridge.tx("regbot madd vel=0.5: lv=20\n");

  // follow line and restart ramp
  bridge.tx("regbot madd vel=0.5, edgel=0, white=1: lv<4, dist=0.2\n");
  bridge.tx("regbot madd vel=0.0: time=0.5\n");
  
  // start this mission
  bridge.tx("regbot start\n");
  cout << "user:# Waiting for" + string(__func__) + "to finish\n" << endl;
  event.waitForEvent(0);
  cout <<  "user:# obstacle " + string(__func__) + " has finished\n" << endl;
}

void seesaw()
{
  bridge.tx("regbot mclear\n");
  event.clearEvents();

  // follow white line, find cross line
  bridge.tx("regbot madd vel=0.3: dist=0.1\n");
  bridge.tx("regbot madd vel=0.3: lv=20, dist=0.5\n");
  bridge.tx("regbot madd vel=0.5, edgel=0, white=1: lv<4, xl>15\n");
  bridge.tx("regbot madd vel=0.0: time=0.5\n");

  // turn into seeseaw
  bridge.tx("regbot madd vel=0.3,tr=0.05: turn=85\n");
  bridge.tx("regbot madd vel=0.0: time=1.5\n");

  // go to midpt of seesaw
  bridge.tx("regbot madd vel=0.5: dist=0.3\n");
  bridge.tx("regbot madd vel=0.5, edgel=0, white=1: dist=0.7, lv<4\n");

  // slowdown near midpt of seesaw
  bridge.tx("regbot madd vel=0.2, edgel=0, white=1: dist=0.6, lv<4\n");
  bridge.tx("regbot madd vel=0.0: time=1.0\n");
  bridge.tx("regbot madd vel=0.3, edgel=0, white=1: lv<4\n");
  bridge.tx("regbot madd vel=0.0: time=1.0\n");

  // go forward until line detected, follow line
  bridge.tx("regbot madd vel=0.5: lv=20\n");
  bridge.tx("regbot madd vel=0.3, tr=0.1: turn=90\n");
  bridge.tx("regbot madd vel=0.0: time=0.5\n");
  bridge.tx("regbot madd vel=0.2, edger=0, white=1: dist=0.2\n");
  
  bridge.tx("regbot madd vel=0.0: time=0.5\n");
  
  // start this mission
  bridge.tx("regbot start\n");
  cout << "user:# Waiting for" + string(__func__) + "to finish\n" << endl;
  event.waitForEvent(0);
  cout <<  "user:# obstacle " + string(__func__) + " has finished\n" << endl;
}

void tunnel()
{
  bridge.tx("regbot mclear\n");
  event.clearEvents();

  // follow white line, find cross line
  bridge.tx("regbot madd vel=0.4: dist=0.1\n");
  bridge.tx("regbot madd vel=0.4: lv=20, dist=0.5\n");
  bridge.tx("regbot madd vel=0.4, edger=0, white=1: dist=1.5, lv<4, xl>15\n");
  
  // turn left towards tunnel
  bridge.tx("regbot madd vel=0.4, tr=0.3: turn=85\n");
  bridge.tx("regbot madd vel=0.4: ir2<0.14\n");
  bridge.tx("regbot madd vel=0.3, tr=0.15:turn=-85\n");

  // go to front door and push open
  bridge.tx("regbot madd vel=0.3: ir2<0.1\n");
  bridge.tx("regbot madd vel=0.0: time=0.5\n");
  bridge.tx("regbot madd vel=0.6: dist=0.1\n");
  bridge.tx("regbot madd vel=0.3: dist=0.05\n");

  // turn left twice into tunnel
  bridge.tx("regbot madd vel=0.3, tr=0.33: turn=90\n");
  bridge.tx("regbot madd vel=0.0: time=0.8\n");
  bridge.tx("regbot madd vel=0.3: dist=0.08\n");
  bridge.tx("regbot madd vel=0.0: time=0.8\n");
  bridge.tx("regbot madd vel=0.3, tr=0.1: turn=90\n");
  bridge.tx("regbot madd vel=0.0: time=2.0\n");

  // go to back door and push open
  bridge.tx("regbot madd vel=0.4: ir2<0.15\n");
  bridge.tx("regbot madd vel=0.5: dist=0.2\n");

  // 1st turn right and go straight
  bridge.tx("regbot madd vel=0.5, tr=0.35: turn=-90\n");
  bridge.tx("regbot madd vel=0.0: time=0.8\n");
  bridge.tx("regbot madd vel=0.5: dist=0.12\n");

  // 2nd turn right and go straight
  bridge.tx("regbot madd vel=0.5, tr=0.3: turn=-90\n");
  bridge.tx("regbot madd vel=0.0: time=0.8\n");
  bridge.tx("regbot madd vel=0.5: dist=0.7\n");

  // 3rd turn right and go straight, close front door
  bridge.tx("regbot madd vel=0.5, tr=0.3: turn=-90\n");
  bridge.tx("regbot madd vel=0.0: time=0.8\n");
  bridge.tx("regbot madd vel=0.5: dist=0.85\n");

  // 4th turn right and go straight
  bridge.tx("regbot madd vel=0.5, tr=0.3: turn=-90\n");
  bridge.tx("regbot madd vel=0.0: time=0.8\n");
  bridge.tx("regbot madd vel=0.5: dist=0.65\n");

  // 5th turn right and go straight, close back door
  bridge.tx("regbot madd vel=0.5, tr=0.3: turn=-80\n");
  bridge.tx("regbot madd vel=0.0: time=0.8\n");
  bridge.tx("regbot madd vel=0.5: dist=0.6\n");
  bridge.tx("regbot madd vel=0.0: time=0.8\n");

  // reverse and return to line
  bridge.tx("regbot madd vel=-0.5: lv=20, dist=2.5\n");
  bridge.tx("regbot madd vel=0.0: time=0.8\n");
  bridge.tx("regbot madd vel=-0.2: dist=0.2\n");
  bridge.tx("regbot madd vel=0.3, tr=0.3: turn=-90\n");
  bridge.tx("regbot madd vel=-0.2, edgel=0, white=1: dist=0.2, lv<4\n");
  bridge.tx("regbot madd vel=0.0: time=0.5\n");
  
  // start this mission
  bridge.tx("regbot start\n");
  cout << "user:# Waiting for" + string(__func__) + "to finish\n" << endl;
  event.waitForEvent(0);
  cout <<  "user:# obstacle " + string(__func__) + " has finished\n" << endl;
}

void axe()
{
  bridge.tx("regbot mclear\n");
  event.clearEvents();
  
  // follow white line, find cross line and turn
  bridge.tx("regbot madd vel=0.3, edgel=0.0, white=1: dist=4, xl>16  \n");
  bridge.tx("regbot madd vel=0.3, tr=0.1: turn=90 \n");

  // travel towards axe
  bridge.tx("regbot madd vel=0.2, edger=0.0, white=1: dist=0.2 \n");
  bridge.tx("regbot madd vel=0.2: dist=0.6 \n");
  bridge.tx("regbot madd vel=0.2, edger=0.0, white=1: dist=0.3  \n");

  // wait for opening
  bridge.tx("regbot madd vel=0: time=0.25  \n");
  bridge.tx("regbot madd :ir2>0.3 \n");
  bridge.tx("regbot madd :ir2<0.3 \n");
  bridge.tx("regbot madd :ir2>0.3 \n");

  // speed through opening
  bridge.tx("regbot madd vl=0.2, edger=0.0, white=1: dist=0.3  \n");
  bridge.tx("regbot madd vel=0.8, edger=0.0, white=1: dist=0.7 \n");
  
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

  // add mission lines
  bridge.tx("regbot madd vel=0.3: dist=0.3\n");
  bridge.tx("regbot madd vel=0.4: dist=0.5, lv=20\n");
  bridge.tx("regbot madd vel=1.2,edger=0.0,white=1:dist=12, lv<3\n");
  bridge.tx("regbot madd vel=0.0:time=0\n");

  // start this mission
  bridge.tx("regbot start\n");
  cout << "Waiting for " + string(__func__) + "to finish" << endl;
  event.waitForEvent(0);
  cout << "user:# obstacle " + string(__func__) + " has finished" << endl;
}

int main(int argc, char **argv)
{
  cout << "# Hello, Robobot user mission starting ..." << endl;
  setup(argc, argv);

  // testMethod();
  gripExample();
  // guillotine();
  // ramp();
  //restartRamp();
  // seesaw();

  // tunnel();
  // axe();
  // fastTrack();

  cout << "# Robobot user mission finished ..." << endl;
  // remember to close camera
  vision.stop();
  sound.say("Mission complete", 0.2);
  while (sound.isSaying())
    sleep(1);
  bridge.tx("regbot mute 1\n");
  return 0;
}
