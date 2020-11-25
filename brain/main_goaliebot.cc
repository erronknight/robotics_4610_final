
#include <iostream>
#include <math.h>
#include <vector>
#include <chrono>
#include <cmath>

#include "robot.hh"
#include "common.hxx"
#include "tasklib.hxx"
#include "taskrt.hxx"

using namespace cv;
using namespace std;

// This is the main task for goaliebot.  There's nothing here yet.
class GoalieMainTask : public RoboTask {
    int twist_count = 0;
public:
    GoalieMainTask() {
    }

    int poll(Robot* robo) override {
	// TODO Make these configurable fields.
	float blocker_angle = 55;
	float desired_side_angle = 90;
	float set_point_left = (desired_side_angle - (blocker_angle)) / 180;

	twist_count += 1;

	if (twist_count < 30) {
	    robo->set_arm_ang(set_point_left);
	} else {
	    robo->set_arm_ang(-set_point_left);
	}

	if (twist_count > 60) {
	    twist_count = 0;
	}

	return TSTATUS_CONTINUE;
    }

    std::string name() override {
	return "GOALIE_MAIN";
    }
};

aistate* robot_state;

// Extra delay to make Valgrind more happy while garbage values are cleaned.
int startup_countdown = 5;

void callback(Robot* robo) {

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
    robot_state->queue_task(new GoalieMainTask());

    Robot robot(argc, argv, callback, string("tankbot1"));
    robot.do_stuff();

    delete robot_state;

    return 0;
}
