
#include <iostream>
#include <thread>
#include <math.h>
#include <vector>
#include <deque>
#include <chrono>
#include <cmath>
#include <string>

#include "robot.hh"

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

#include <mutex>

/* Surface to store current scribbles */
std::mutex mx2;

using namespace cv;
using namespace std;

void
kick_kallback(Robot* robot) {
//  robot->set_arm_ang(0.0f); // completely forward
  // robot->set_arm_ang(0.5f); // t-pose
  // robot->set_arm_ang(1.0f); // backwards

  robot->set_kick_val(1.0f); // completely extended
  // robot->set_kick_val(0.0f); // stowed
}

int twist_count = 0;

void
goalie_callback(Robot* robot) {
  float blocker_angle = 55;
  float desired_side_angle = 90;
  float set_point_left = (desired_side_angle - (blocker_angle)) / 180;
  
  twist_count += 1;
  
  if (twist_count < 30) {
    robot->set_arm_ang(set_point_left);
  } else {
    robot->set_arm_ang(-set_point_left);
  }
  
  if (twist_count > 60) {
    twist_count = 0;
  }
}


void
robotLoop(Robot* robot) {
    robot->do_stuff();
}

#include <X11/Xlib.h>
int
main(int argc, char* argv[]) {
    XInitThreads();
    Robot kicker(argc, argv, kick_kallback, string("tankbot0"));
    std::thread kickerThread(robotLoop, &kicker);

    Robot goalie(argc, argv, goalie_callback, string("tankbot1"));
    std::thread goalieThread(robotLoop, &goalie);
    
    pause();
    
    return 0;
}
