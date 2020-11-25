
#include <iostream>
#include <thread>
#include <math.h>
#include <vector>
#include <deque>
#include <chrono>
#include <cmath>
#include <string>
#include <mutex>

#include "robot.hh"
#include "common.hxx"
#include "tasklib.hxx"

/* Surface to store current scribbles */
std::mutex mx2;

using namespace cv;
using namespace std;

// This is the main task for kickerbot.  I just copied the poll impl from what
// was in the callback when I got to this part.
class KickerMainTask : public RoboTask {
public:
    KickerMainTask() {
    }

    int poll(Robot* robo) override {
	// robot->set_arm_ang(0.0f); // completely forward
	// robot->set_arm_ang(0.5f); // t-pose
	// robot->set_arm_ang(1.0f); // backwards
	robo->set_kick_val(1.0f); // completely extended
	// robot->set_kick_val(0.0f); // stowed
	return TSTATUS_CONTINUE;
    }

    std::string name() override {
	return "KICKER_MAIN";
    }
};

aistate* robot_state;

// Extra delay to make Valgrind more happy while garbage values are cleaned.
int startup_countdown = 5;

void callback(Robot* robo ) {

    if (startup_countdown > 0) {
        startup_countdown--;
        return;
    }

    aistate* state = robot_state;

    // High level wrapper logic for the task subsystem.
    if (!state->active) {
        robo->done();
        return;
    }

    if (state->has_pending_tasks()) {
        do_poll_tasks(robo, state);
    } else {
        cout << "[all tasks finished]" << endl;
        state->active = false;
    }

}

int main(int argc, char** argv) {

    robot_state = new aistate;
    robot_state->active = true;
    robot_state->queue_task(new KickerMainTask());

    Robot robot(argc, argv, callback, string("tankbot0"));
    robot.do_stuff();

    delete robot_state;

    return 0;
}
