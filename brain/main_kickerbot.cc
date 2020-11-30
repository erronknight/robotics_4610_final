
#include <iostream>
#include <math.h>
#include <vector>
#include <chrono>
#include <cmath>
#include <string>

#include "robot.hh"
#include "common.hxx"
#include "tasklib.hxx"
#include "taskrt.hxx"
#include "search.hh"
#include <math.h>

using namespace std;


aistate *robot_state;

float turn_speed = 2.5;
float drive_speed = 3.5;

bool collected = false;

void set_default_state(Robot *robo) {
    robo->set_kick_val(0.0);
    robo->set_arm_ang(0.5);
}

class KickerDriveTask : public RoboTask {
public:
    KickerDriveTask() {}
    
    int poll(Robot *robo) override {
        float dist = get_distance_from_ball(robo);
        if (dist > 1) {
            set_default_state(robo);
            
            float angle = get_direction_of_ball(robo) * 50;
            float max = std::max(drive_speed + angle, drive_speed - angle);
            float factor = drive_speed / max;
            
            robo->set_vel((drive_speed + angle) * factor, (drive_speed - angle) * factor);
            return TSTATUS_CONTINUE;
        } else if (dist == -1) {
            return TSTATUS_ABORT;
        }
        
        robo->set_arm_ang(0.1);
        collected = true;
        return TSTATUS_DONE;
    }
    
    std::string name() override {
        return "KICKER_DRIVE";
    }
};

class KickerTurnTask : public RoboTask {
public:
    KickerTurnTask() {}
    
    int poll(Robot *robo) override {
        if (!turn_to_ball(robo, 0.1)) {
            set_default_state(robo);
            return TSTATUS_CONTINUE;
        }
        
        return TSTATUS_DONE;
    }
    
    std::string name() override {
        return "KICKER_TURN";
    }
};


class KickerOpenArmTask : public TaskDelay {
public:
    KickerOpenArmTask(uint64_t ms, bool st) : TaskDelay(ms, st) {}
    
    int poll(Robot *robo) override {
        robo->set_arm_ang(0.5);
        return TaskDelay::poll(robo);
    }
    
    std::string name() override {
        return "KICKER_OPEN_ARM_TASK";
    }
};


class KickerShootTask : public TaskDelay {
public:
    KickerShootTask(uint64_t ms, bool st) : TaskDelay(ms, st) {}
    
    int poll(Robot *robo) override {
        int result = TaskDelay::poll(robo);
        
        if (result == TSTATUS_CONTINUE) {
            robo->set_kick_val(1.0);
        } else {
            robo->set_kick_val(0.0);
        }
        
        return result;
    }
    
    std::string name() override {
        return "KICKER_OPEN_ARM_TASK";
    }
};


class KickerSearchTask : public RoboTask {
public:
    KickerSearchTask() {}
    
    int poll(Robot *robo) override {
        if (!search_for_ball(robo)) {
            set_default_state(robo);
            return TSTATUS_CONTINUE;
        }
        return TSTATUS_DONE;
    }
    
    std::string name() override {
        return "KICKER_SEARCH";
    }
};


class KickerCollectTask : public RoboTask {
public:
    KickerCollectTask() {}
    
    int poll(Robot *robo) override {
        if (!collected) {
            robot_state->queue_task(new KickerSearchTask());
            robot_state->queue_task(new KickerTurnTask());
            robot_state->queue_task(new KickerDriveTask());
        } else {
            return TSTATUS_DONE;
        }
        
        return TSTATUS_CONTINUE;
    }
    
    std::string name() override {
        return "KICKER_COLLECT";
    }
};

void determine_shot_position(Vec2f robot_pos, float shot_len, Vec2f goal_loc, Vec2f &dest, Vec2f &shot_loc) {

    float angles[3] = {-35, 0, 35};
    Vec2f shot_offsets[3] = {Vec2f(-1, -0.8), Vec2f(0, -1), Vec2f(1, -0.8)};
    Vec2f shot_loc_offset = Vec2f(-1, -0.8);
    float shot_angle;

    Vec2f offset;
    Vec2f dest_tmp;
    Vec2f offset_tmp;
    float tmp_min_dist = 100;
    for(int i=0; i < 3; i++) {
        shot_angle = angles[i];
        offset_tmp = Vec2f(shot_len * cos((180 - shot_angle) * M_PI / 180.0f),
                         shot_len * sin((180 - shot_angle) * M_PI / 180.0f));
        dest_tmp = goal_loc + offset;
        if ((abs(dest_tmp.x - robot_pos.x) + abs(dest_tmp.y - robot_pos.y)) <= tmp_min_dist) {
            tmp_min_dist = (abs(dest_tmp.x - robot_pos.x) + abs(dest_tmp.y - robot_pos.y));
            offset = offset_tmp;
            shot_loc_offset = shot_offsets[i];
        }
    }

    // offset = Vec2f(shot_len * cos((180 - shot_angle) * M_PI / 180.0f),
    //                      shot_len * sin((180 - shot_angle) * M_PI / 180.0f));
    dest = goal_loc + offset;
    shot_loc = goal_loc + shot_loc_offset;
}

// This is the main task for kickerbot.  I just copied the poll impl from what
// was in the callback when I got to this part.
class KickerMainTask : public RoboTask {
public:
    KickerMainTask() {}
    
    int poll(Robot *robo) override {
        float shot_length = 5;
        float shot_angle = -35;
        
        if (!collected) {
            robot_state->queue_task(new KickerCollectTask());
        } else {
            Vec2f offset = Vec2f(shot_length * cos((180 - shot_angle) * M_PI / 180.0f),
                                 shot_length * sin((180 - shot_angle) * M_PI / 180.0f));
            Vec2f goal_location = Vec2f(6.0, 0.0);
            Vec2f destination = goal_location + offset;
            Vec2f shot_spot = goal_location + Vec2f(-1, -0.8);
            
            determine_shot_position(Vec2f(robo->pos_x, robo->pos_y), shot_length, goal_location, destination, shot_spot);

            float tgt_hdg, dx, dy;
            // See macro above.
            calc_target_heading(destination, shot_spot, dx, dy, tgt_hdg);
            
            robot_state->queue_task(new TaskMoveTowards(destination, drive_speed, 0.05));
            robot_state->queue_task(new TaskTurnTo(tgt_hdg));
            robot_state->queue_task(new KickerOpenArmTask(500, true));
            robot_state->queue_task(new KickerShootTask(100, true));
            collected = false;
        }
        
        return TSTATUS_CONTINUE;
    }
    
    std::string name() override {
        return "KICKER_MAIN";
    }
};


// Extra delay to make Valgrind more happy while garbage values are cleaned.
int startup_countdown = 5;

void callback(Robot *robo) {
    
    if (startup_countdown > 0) {
        startup_countdown--;
        return;
    }
    
    aistate *state = robot_state;
    
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

#include <X11/Xlib.h>

using namespace cv;
using namespace std;


int main(int argc, char **argv) {
    XInitThreads();
    robot_state = new aistate;
    robot_state->active = true;
    robot_state->queue_task(new KickerMainTask());
    Robot robot(argc, argv, callback, string("tankbot0"));
    
    robot.do_stuff();
    
    delete robot_state;
    return 0;
}
