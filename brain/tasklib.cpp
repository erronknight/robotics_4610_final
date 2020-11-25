#include "common.hxx"
#include "tasklib.hxx"

void print_ad_test(float f, float t) {
    float diff = angle_diff(f, t);
    cout << "from = " << f << " to = " << t << " diff = " << diff << endl;
}

// This was for debugging.
void test_angle_diff() {
    print_ad_test(1, -1);
    print_ad_test(-1, 1);
    print_ad_test(1.5, -3);
    print_ad_test(3, -1.5);
    print_ad_test(3, -3);
    print_ad_test(-3, 3);
    print_ad_test(0, -3);
    print_ad_test(-3, 0);
    print_ad_test(0, 3);
    print_ad_test(3, 0);
    print_ad_test(-M_PI + 1, M_PI - 1);
    print_ad_test(M_PI - 1, -M_PI + 1);
}

/**
 * Helper to go in a direction for a duration.
 */
void do_go_heading(Robot* robo, float base_speed, float hdg, bool do_print) {
    float hdg_off = angle_diff(robo->pos_t, hdg);
    float turn_force = fminf(fmaxf(hdg_off, -1), 1) * 0.45;
    float turn_mag = turn_force * base_speed;
    //turn_mag = turn_mag >= 0.001 ? turn_mag : 0;
    if (do_print) {
        cout << "hdgoff = " << hdg_off
            << ", turnmag = " << turn_mag << endl;
    }

    robo->set_vel(base_speed - turn_mag, base_speed + turn_mag);
}

TaskDelay::TaskDelay(uint64_t ms, bool st) {
    end_time = ms;
    stop = st;
}

void TaskDelay::init(Robot* robo) {
    end_time += get_now_time();
    if (stop) {
        robo->set_vel(0, 0);
    }
}

int TaskDelay::poll(Robot* robo) {
    if (get_now_time() < end_time) {
        return TSTATUS_CONTINUE;
    } else {
        return TSTATUS_DONE;
    }
}

std::string TaskDelay::name() {
    if (stop) {
        return "DELAY:STATIC";
    } else {
        return "DELAY:DYNAMIC";
    }
}

TaskTurnTo::TaskTurnTo(float th) {
    target_hdg = th;
    measured_hdg = 0;
    ok_thresh = 0.01;
}

TaskTurnTo::TaskTurnTo(float th, float okt) {
    target_hdg = th;
    measured_hdg = 0;
    ok_thresh = okt;
}

int TaskTurnTo::poll(Robot* robo) {
    float hdg_off = angle_diff(robo->pos_t, target_hdg);
    if (step % 10 == 0) {
        cout << "current heading = " << robo->pos_t << " off = " << hdg_off << endl;
    }
    step++;

    // Within some range we can say this is good enough.
    if (abs(hdg_off) < ok_thresh) {
        return TSTATUS_DONE;
    }

    float right_speed = 1.5; // left is this times -1.

    // If we're close then slow down.
    if (abs(hdg_off) < ok_thresh * 2) right_speed = 0.7;

    // If reversing, flip steering.
    if (hdg_off < 0) right_speed *= -1;

    robo->set_vel(right_speed * -1, right_speed);
    return TSTATUS_CONTINUE;
}

std::string TaskTurnTo::name() {
    return "TURN_TO";
}

/*
class DebugPrintPosTask : public RoboTask {
public:
    int poll(Robot* robo) override {
        cout << "x = " << robo->pos_x << ", y = " << robo->pos_y << endl;
        return TSTATUS_CONTINUE;
    }
};

class DebugPrintLidarTask : public RoboTask {
public:
    int poll(Robot* robo) override {
        cout << "=========" << endl;
        for (auto hit : robo->ranges) {
            cout << "h(" << hit.angle << ", " << hit.range << ")" << endl;
        }

        return TSTATUS_CONTINUE;
    }
};
*/

TaskGo::TaskGo(float th, float bv) {
    target_hdg = th;
    base_vel = bv;
    // 0.21 is roughly the 2x the average polling rate.
    pidloop = new PID(0.21, bv * 0.8, bv * -0.8, 10, 0.35, 0.1);
}

TaskGo::~TaskGo() {
    delete pidloop;
}

int TaskGo::poll(Robot* robo) {

    // How we signal we're done.
    if (step == -1) {
        return TSTATUS_DONE;
    }

    // Jank thing to make the loop more stable.
    step = 1 - step;
    if (step == 0) {
        return TSTATUS_CONTINUE;
    }

    uint64_t now = get_now_time();

    float hdg_off = angle_diff(robo->pos_t, target_hdg);
    float pidres = pidloop->calculate(0, hdg_off);
    cout << "off = " << hdg_off << ", pidres = " << pidres << endl;

    lasttime = now;
    robo->set_vel(base_vel + pidres, base_vel - pidres);

    return TSTATUS_CONTINUE;
}

std::string TaskGo::name() {
    return "GO";
}

void TaskGo::update_heading(float nhdg) {
    target_hdg = nhdg;
}

void TaskGo::stop() {
    step = -1;
}

TaskGoDur::TaskGoDur(float bv, float hdg, uint64_t et) {
    base_vel = bv;
    heading = hdg; // ignored for now
    end_time = et;
}

void TaskGoDur::init(Robot* robo) {
    end_time += get_now_time();
    robo->set_vel(base_vel, base_vel);
}

int TaskGoDur::poll(Robot* robo) {
    return get_now_time() > end_time ? TSTATUS_DONE : TSTATUS_CONTINUE;
}

std::string TaskGoDur::name() {
    return "GO_DUR";
}

TaskMoveTowards::TaskMoveTowards(Vec2i start_tgt, float vel, float dist) {
    target_pos = start_tgt;
    base_vel = vel;
    end_dist = dist;
    move_delegate = NULL;
}

// Absolutely terrible use of a macro.
#define calc_target_heading(robo, tgt) \
    float dx = tgt.x - robo->pos_x; \
    float dy = tgt.y - robo->pos_y; \
    float tgt_hdg = atan2(dy, dx);

int TaskMoveTowards::poll_inactive(Robot* robo) {

    // See macro above.
    calc_target_heading(robo, target_pos);
    float dt = angle_diff(robo->pos_t, tgt_hdg);
    cout << "dx = " << dx << " dy = " << dy << " theta = " << tgt_hdg << " dt = " << dt << endl;

    // Check if we're close enough.
    if (dx * dx + dy * dy < end_dist * end_dist && end_dist > 0) {
        step = 10;
        return TSTATUS_INTERRUPT;
    }

    switch (step) {

    // Normal cases.
    case 1:
        return TSTATUS_CONTINUE;

    // Now just keep the right heading.
    case 2:
    {
        move_delegate->update_heading(tgt_hdg);
        return TSTATUS_CONTINUE;
    }

    }

    return TSTATUS_ABORT;
}

int TaskMoveTowards::poll(Robot* robo) {

    // See macro above.
    calc_target_heading(robo, target_pos);

    // Check if we're cloe enough.
    if (dx * dx + dy * dy < end_dist * end_dist && end_dist > 0) {
        return TSTATUS_DONE;
    }

    switch (step) {

    // First thing just turn towards the target.
    case 0:
    {
        step++;
        auto tt = new TaskTurnTo(tgt_hdg, 0.1);
        ai->queue_task(tt);
        return TSTATUS_CONTINUE;
    }

    // Now we can actually start moving towards the target.
    case 1:
    {
        step++;
        move_delegate = new TaskGo(tgt_hdg, base_vel);
        ai->queue_task(move_delegate);
        return TSTATUS_CONTINUE;
    }

    // This is for we interrupted and want to exit now.
    case 10:
    return TSTATUS_DONE;

    }

    // Did something bad.
    return TSTATUS_ABORT;
}

std::string TaskMoveTowards::name() {
    std::string buf = "MOVE(";
    buf << target_pos.x << ", " << target_pos.y << ")";
    return buf;
}

void TaskMoveTowards::update_target(Vec2i new_tgt) {
    target_pos = new_tgt;
}

void print_stack_trace(aistate* state) {
    for (int i = state->task_stack.size() - 1; i >= 0; i--) {
        TaskEntry* te = state->task_stack[i];
        cout << "\ttask " << i << ": "
            << te->task->name() << "@" << te->task->step
            << " " << statenamev[te->state];

	if (te->comment != NULL) {
	    cout << " (" << *te->comment << ")";
        }

        cout << endl;
    }
}

void do_abort(Robot* robo, aistate* state) {
    cout << "ABORTING" << endl << "TASK TRACE:" << endl;
    state->active = false;
    print_stack_trace(state);

    // Kill movement.
    robo->set_vel(0, 0);
}

void do_poll_tasks(Robot* robo, aistate* state) {

    // First pop any tasks that have completed.
    while (state->task_stack.size() > 0
        && state->task_stack.back()->state == TSTATE_DONE) {
        TaskEntry* te = state->task_stack.back();
        state->task_stack.pop_back();
        delete te->task;
        delete te;
    }

    // Now add all the tasks that were queued onto the new stack.  Don't init
    // them yet because we only do that one at a time.
    int added_tasks = 0;
    while (state->fresh_tasks.size() > 0) {
        TaskEntry* te = state->fresh_tasks.back();
        state->fresh_tasks.pop_back();
        state->task_stack.push_back(te);
        added_tasks++;
    }

    if (added_tasks > 0) {
	cout << "[added " << added_tasks << " fresh tasks to stack]" << endl;
    }

    TaskEntry* cur_task = state->task_stack.back();

    // First we want to poll_inactive all the other tasks first.
    bool interrupted = false;
    for (size_t i = 0; i < state->task_stack.size() - 1 && !interrupted; i++) {
	TaskEntry* at = state->task_stack[i];

        // Don't poll_inactive tasks that haven't been inited yet or have exited.
        if (at->state != TSTATE_ACTIVE) {
            continue;
        }

        int ipoll_res = at->task->poll_inactive(robo);

        switch (ipoll_res) {
        case TSTATUS_DONE:
            {
                // We'll clean these up later.
                cout << "[task " << at->task->name() << " background completed (idx " << i << ")]" << endl;
                at->state = TSTATE_DONE;
                break;
            }
        case TSTATUS_CONTINUE:
            break;
        case TSTATUS_ABORT:
            do_abort(robo, state);
            return;
        case TSTATUS_INTERRUPT:
            {
                cout << "[task " << i << " " << at->task->name() << " interrupted]" << endl;

                // Change what the current task is supposed to be.
                cur_task = at;
                interrupted = true;

                // Go through and delete all the tasks after this one.
                size_t rem = state->task_stack.size() - i - 1;
                for (size_t j = 0; j < rem; j++) {
                    TaskEntry* te = state->task_stack.back();
                    state->task_stack.pop_back();
                    cout << "[[clearing an interrupted " << te->task->name() << " task]]" << endl;

                    delete te->task;
                    delete te;
                }

                break;
            }
        }
    }

    // Activate the task if we haven't already.  If we're coming from an
    // interrupt then this will never be true.
    if (cur_task->state == TSTATE_NEW) {
        cout << "[initing task " << cur_task->task->name() << "]";
        if (cur_task->comment != NULL) cout << " (" << *cur_task->comment << ")";
        cout << endl;
        cur_task->task->init(robo);
        cur_task->state = TSTATE_ACTIVE;
    }

    // Invoke the topmost task.
    //cout << "[polling " << cur_task->task->name() << " (qsize = " << state->task_stack.size() << ")]" << endl;
    int poll_res = cur_task->task->poll(robo);
    //cout << "[poll result " << poll_res << "]" << endl;

    switch (poll_res) {
    case TSTATUS_INTERRUPT:
        cout << "[warning: current task requested interrupt, ignoring]" << endl;
    case TSTATUS_DONE:
        {
            // Pull the task off the top of the stack.
            cur_task->state = TSTATE_DONE;

            cout << "[task " << cur_task->task->name() << " completed]";
            if (cur_task->comment != NULL) cout << " (" << *cur_task->comment << ")";
            cout << endl;

            //print_stack_trace(state); // TODO Remove.
            state->task_stack.pop_back();

            delete cur_task->task;
            delete cur_task;

            // Want to kill our velocity now.  Just to not leave things unclean.
            robo->set_vel(0, 0);
            break;
        }
    case TSTATUS_CONTINUE:
        break;
    case TSTATUS_ABORT:
        {
            do_abort(robo, state);
            return;
        }
    }

}

