
#include <iostream>
#include <thread>
#include <math.h>
#include <vector>
#include <deque>
#include <chrono>
#include <cmath>

#include "robot.hh"

#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"

#include <mutex>

/* Surface to store current scribbles */
std::mutex mx2;

using namespace cv;
using namespace std;

void
callback(Robot* robot) {
//  robot->set_arm_ang(0.0f); // completely forward
  // robot->set_arm_ang(0.5f); // t-pose
  // robot->set_arm_ang(1.0f); // backwards

  robot->set_kick_val(1.0f); // completely extended
  // robot->set_kick_val(0.0f); // stowed
}

#include <X11/Xlib.h>
int
main(int argc, char* argv[]) {
    XInitThreads();
    Robot robot(argc, argv, callback);
    robot.do_stuff();
    return 0;
}
