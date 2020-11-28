
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


#define TARGET_ANGLE    45
#define BALLERANCE      M_PI * 10.0 / 180.0     // Tolerance for the ball in the middle of frame


// This is the main task for goaliebot.  There's nothing here yet.
class GoalieMainTask : public RoboTask {
    int twist_count = 0;

public:
    GoalieMainTask() {
    }

    int poll(Robot* robo) override {
	    //// TODO Make these configurable fields.
	    //float blocker_angle = 55;
	    //float desired_side_angle = 90;
	    //float set_point_left = (desired_side_angle - (blocker_angle)) / 180;

	    //twist_count += 1;

	    //if (twist_count < 30) {
	    //    robo->set_arm_ang(1);
	    //} else {
	    //    robo->set_arm_ang(0);
	    //}


        // Initial - No ball in frame
        if (robo->frame.empty()) {
            cout << "GOALIE FRAME EMPTY" << endl;
            return TSTATUS_CONTINUE;
        }


        float ball_ang = get_direction_of_ball(robo->frame);
        float ball_dist = get_distance_from_ball(robo->frame);

        cout << "[" << ball_ang << ", " << ball_dist << "]: " << endl;

        
        // Ball not in frame - Reset
        if (ball_dist < 0) {
            cout << "GOALIE BALL NOT IN FRAME" << endl;
            robo->set_arm_ang(-TARGET_ANGLE);
            return TSTATUS_CONTINUE;
        }

        // Ball close! - Fling your arms!!
        else if (ball_dist < 1.5) {
            cout << "BALL CLOSE! FLINGING ARMS" << endl;
            robo->set_arm_ang(TARGET_ANGLE);
            return TSTATUS_CONTINUE;
        }

        // Move back and forth in the direction of the ball
        if (abs(ball_ang) > BALLERANCE) {
            if (abs(robo->pos_y) > 1.25) {
                cout << "MOVING TO BALL" << endl; 
                float vel = (ball_ang < 0) ? -1.5 : 1.5;
                robo->set_vel(vel, vel);
                return TSTATUS_CONTINUE;
            }
        }

        cout << "GOALIE DEFAULT" << endl;

        robo->set_arm_ang(-TARGET_ANGLE);
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
