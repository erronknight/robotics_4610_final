
#include <iostream>
#include <math.h>
#include <vector>
#include <chrono>
#include <cmath>

#include "robot.hh"
#include "common.hxx"
#include "tasklib.hxx"
#include "taskrt.hxx"
#include "search.hh"

using namespace cv;
using namespace std;

aistate* robot_state;

// This is the main task for goaliebot.
class GoalieMainTask : public RoboTask {
private:
    int twist_count = 0;

    void strafe(Robot* robot) {
        static auto strafe_vel = 4;
        static int tics = 0;

        if (tics++ == 0) {
            strafe_vel *= -1;
        }
        tics %= 200;

        robot->set_vel(strafe_vel, strafe_vel);
    }

public:
    GoalieMainTask() {
    }

    int poll(Robot* robo) override {
	// TODO Make these configurable fields.
	float blocker_angle = 55;
	float desired_side_angle = 90;
	float set_point_left = (desired_side_angle - (blocker_angle)) / 180;

	twist_count += 1;

    float ball_ang = get_direction_of_ball(robo->frame);
    float ball_dist = get_distance_from_ball(robo->frame);
    bool ball_in_sight = ball_dist > 0 && !isinf(ball_dist);

    if (ball_in_sight) {
        if (ball_dist < 600) {
            robo->set_vel(0, 0);
            return TSTATUS_CONTINUE;
        }
    } else {
        this->strafe(robo);
    }

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
