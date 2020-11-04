
#include <iostream>
#include <thread>
#include <mutex>
#include <math.h>

#include "common.hxx"
#include "tasklib.hxx"
#include "robot.hh"

#include "pid.hxx"
#include "nav.hxx"

extern "C" {
#include "gfx.h"
}

#define VIZ_WIDTH 400
#define VIZ_HEIGHT 400
#define VIZ_SCALE 5.0

aistate* robot_state;
std::mutex* draw_mutex;

/**
 * If this is positive then turn out, if negative turn in.  This is agnostic to
 * as to which side we're following the wall on.  It gets more information than
 * it needs for what it does now, but we might make this more advanced in the
 * future.
 *
 * This isn't refer to specific angles, just sorta like, proportions?
 *
 * Range is [-1, 1].
 */
float calc_wall_prox_off(float front, float side, float back) {

    if (front > 5) {
        return -1;
    }

    if (front < 0.75) {
        return 1;
    }

    // Normalize to make the math easier.
    float n_front = front / M_SQRT2;
    float n_back = back / M_SQRT2;

    float fb_diff = clamp(n_back - n_front, -1, 1);

    if (fabsf(side - 1) < 0.1) {
        return 0;
    }

    // ehh?
    return clampf((1 - side), -1, 1);
}

/**
 * Checks to see if we're far away from anything nearby.  Used to switch into
 * a wander mode.
 */
bool check_lost(Robot* robo) {
    for (auto hit : robo->ranges) {
        if (hit.range < 999) {
            return false;
        }
    }
    return true;
}

void shitty_box(int x, int y, int w, int h) {
    for (int i = x; i < x + w; i++) {
        for (int j = y; j < y + h; j++) {
            gfx_point(i, j);
        }
    }
}

void draw_known_world(Robot* robo, aistate* ai, std::vector<Vec2i>& updated) {

    //cout << "drawing" << endl;

    draw_mutex->lock();
    //gfx_lock();

	//gfx_color(0, 0, 0);
	gfx_clear_color(0, 0, 0);
	
	gfx_clear();

	Vec2i rpos((int) roundf(robo->pos_x), (int) roundf(robo->pos_y));

	// First draw the map of where things are.
	Grid* g = ai->nav.get_grid();
	for (int x = -30; x < 31; x++) {
	for (int y = -30; y < 31; y++) {
	//for (int i = 0; i < updated.size(); i++) { int x = updated[i].x; int y = updated[i].y;

        // Figure out the color based on the weight.
        float weight = g->get_cell(x, y);
        int color = (int) floorf(weight * 255 / 3.0);
        if (color < 16) {
            continue;
        }

        if (weight < BASE_OCCUPIED_THRESH) {
            gfx_color(0, 0, color);
        } else {
            gfx_color(color, color, color);
        }

        // And then just place the pixel.
        int scr_x = (VIZ_WIDTH / 2) + -y * VIZ_SCALE;
        int scr_y = (VIZ_HEIGHT / 2) + -x * VIZ_SCALE;
        gfx_box(scr_x, scr_y, VIZ_SCALE, VIZ_SCALE);

    }
	}

	// Now draw the current path.
	std::vector<Vec2i>* psteps = ai->nav.get_path();
	gfx_color(0, 255, 0);
	for (int i = 0; i < psteps->size(); i++) {
	    Vec2i start = i == 0 ? rpos : (*psteps)[i - 1];
	    Vec2i end = (*psteps)[i];
	    
        int s_scr_x = (VIZ_WIDTH / 2) + -start.y * VIZ_SCALE + (VIZ_SCALE / 2);
        int s_scr_y = (VIZ_HEIGHT / 2) + -start.x * VIZ_SCALE + (VIZ_SCALE / 2);
        int e_scr_x = (VIZ_WIDTH / 2) + -end.y * VIZ_SCALE + (VIZ_SCALE / 2);
        int e_scr_y = (VIZ_HEIGHT / 2) + -end.x * VIZ_SCALE + (VIZ_SCALE / 2);

        gfx_line(s_scr_x, s_scr_y, e_scr_x, e_scr_y);
	}

	// And draw the robot.
	int r_scr_x = (VIZ_WIDTH / 2) + -robo->pos_y * VIZ_SCALE;
	int r_scr_y = (VIZ_HEIGHT / 2) + -robo->pos_x * VIZ_SCALE;
	gfx_color(255, 255, 0);
	//gfx_clear_color(255, 255, 0);
	gfx_box(r_scr_x, r_scr_y, VIZ_SCALE / 2, VIZ_SCALE / 2);
	
	// And draw the current target.
	Vec2i ct = ai->nav.get_current_target();
	int t_scr_x = (VIZ_WIDTH / 2) + -ct.y * VIZ_SCALE + (VIZ_SCALE / 2);
	int t_scr_y = (VIZ_HEIGHT / 2) + -ct.x * VIZ_SCALE + (VIZ_SCALE / 2);
	gfx_color(255, 0, 0);
	gfx_box(t_scr_x, t_scr_y, VIZ_SCALE / 2, VIZ_SCALE / 2);

	gfx_flush();
    //gfx_unlock();
	draw_mutex->unlock();
	
}

void draw_samples(Robot* robo) {

    gfx_color(255, 255, 255);

    // canv_theta is with 0 being right.
    float theta = robo->pos_t;
    float world_theta = theta < 0 ? (2 * M_PI) + theta : theta;
    float canv_theta = world_theta + (M_PI / 2);

    // +y is up +x is right.
    float canv_x = robo->pos_y * -1;
    float canv_y = robo->pos_x;

    int drew = 0;
    for (auto hit : robo->ranges) {
        // If there's no hit then just skip it.
        if (hit.range > 5) {
            continue;
        }

        // Do some math to figure out where to put it.
        float hit_theta = canv_theta + hit.angle;
        float hit_canv_x = canv_x + hit.range * cosf(hit_theta);
        float hit_canv_y = canv_y + hit.range * sinf(hit_theta);
        float scr_x = (VIZ_WIDTH / 2) + VIZ_SCALE * hit_canv_x + (VIZ_SCALE / 2);
        float scr_y = (VIZ_HEIGHT / 2) + -VIZ_SCALE * hit_canv_y + (VIZ_SCALE / 2);

        gfx_point(scr_x, scr_y);
        drew++;
    }

    gfx_flush();
    //cout << "Drew " << drew << " points" << endl;
}

#define UPNAV_STEP_INC 0.7
#define UPNAV_MAX_RANGE 2

Vec2f update_nav(Robot* robo, NavController* nav, std::vector<Vec2i>& chgs) {
    // The easy part is updating the next waypoint.
    Vec2f rpos(robo->pos_x, robo->pos_y);
    nav->update_cur_pos(rpos);
    nav->update_cell(rpos.to_vec2i(), false); // we know we're not a wall

    // Mark all the cells.
    for (auto hit : robo->ranges) {
        
        if (hit.range < 999) {
            // If we hit something then mark it.
            float hwx = robo->pos_x + hit.range * cos(hit.angle);
            float hwy = robo->pos_y + hit.range * sin(hit.angle);
            Vec2i hpos = Vec2f(hwx, hwy).to_vec2i();
            nav->update_cell(hpos, true);
            chgs.push_back(hpos);
            
            /*if (hit.range > 1) {
                float wx = robo->pos_x + cos(hit.angle);
                float wy = robo->pos_y + sin(hit.angle);
                Vec2i cpos((int) roundf(wx), (int) roundf(wy));
                nav->update_cell(cpos, false);
            }*/
            
        } else {
            // Clear out what's in our immediate vicinity.
            float wx = robo->pos_x + 0.5 * cos(hit.angle);
            float wy = robo->pos_y + 0.5 * sin(hit.angle);
            Vec2i cpos = Vec2f(wx, wy).to_vec2i();
            nav->update_cell(cpos, false);
        }

    }

    uint64_t before = get_now_time();
    bool ok_path = nav->validate_path();
    uint64_t after = get_now_time();
    Vec2i tgt = nav->get_current_target();
    if (!ok_path) {
        cout << "NAV: found new waypt to target: " << tgt.x << "," << tgt.y << endl;
        cout << "NAV: recalculating took " << (after - before) << " ms" << endl;
    }

    return rpos;
}

/**
 * Based on the sensors, tries to back up or do something to just get away from
 * the walls that we might be stuck on.  Might not do anything if we're already
 * away.
 *
 * Might also get stuck in infinite loops in tight areas.
 */
class TaskAvoidWalls : public RoboTask {
    float proximity;
    float avoid_vel = 2;

public:
    TaskAvoidWalls(float prox) {
        proximity = prox;
    }

    int poll(Robot* robo, aistate* ai) override {
        
        // First check if we're ok.
        bool all_ok = true;
        for (auto hit : robo->ranges) {
            if (hit.range < proximity) {
                all_ok = false;
                break;
            }
        }
        if (all_ok) {
            return TSTATUS_DONE;
        }
        
        // Now we have to decide how we want to turn.
        DistSamples dists(robo);
        
        // First check in front, that's where it's most likely to anyways.
        if (dists.fwd_dist < proximity) {
            robo->set_vel(-avoid_vel, -avoid_vel);
            return TSTATUS_CONTINUE;
        }
        
        // Front right, back up to the left.
        if (dists.frt_dist < proximity) {
            robo->set_vel(avoid_vel * -0.5, avoid_vel * -1.5);
            return TSTATUS_CONTINUE;
        }
        
        // Front left, back up to the right.
        if (dists.flt_dist < proximity) {
            robo->set_vel(avoid_vel * -1.5, avoid_vel * -0.5);
            return TSTATUS_CONTINUE;
        }
        
        // Directly back we can't really do, so just do it like this.
        if (dists.brt_dist < proximity || dists.blt_dist < proximity) {
            robo->set_vel(avoid_vel, avoid_vel);
            return TSTATUS_CONTINUE;
        }
        
        return TSTATUS_DONE;        
    }
    
    std::string name() override {
        return "AVOID_WALLS";
    }

};

#define FN_NORMAL 0
#define FN_NEXT_STEP 1
#define FN_TURN_WAIT 2
#define FN_STOP 3

class TaskFollowNav : public RoboTask {
    TaskMoveTowards* move_delegate = NULL;

public:
    TaskFollowNav() {
        
    }
    
    void init(Robot* robo, aistate* ai) override {
        // TODO Set up a buffer where we store the last handful of location
        // samples and average them.
        step = FN_NORMAL;
        ai->nav.validate_path();
    }
    
    int poll_inactive(Robot* robo, aistate* ai) override {
        // FIXME This is actually the second time we're doing this because I'm
        // stupid and forgot.  Mostly harmless.  See `update_nav()`.
        Vec2f rpos(robo->pos_x, robo->pos_y);
        bool new_target = ai->nav.update_cur_pos(rpos);
        bool path_ok = ai->nav.validate_path();
        bool make_sure = ai->nav.validate_path();
        if (!make_sure) {
            cout << "{{{we fucked up somewhere}}}" << endl;
        }
        
        if (ai->nav.is_done()) {
            step = FN_STOP;
            return TSTATUS_INTERRUPT;
        }

        if ((new_target || !path_ok) && step == FN_NORMAL) {
            step = FN_NEXT_STEP;
            move_delegate = NULL;
            return TSTATUS_INTERRUPT;
        }

        return TSTATUS_CONTINUE;
    }

    int poll(Robot* robo, aistate* ai) override {
        if (step == FN_STOP || ai->nav.is_done()) {
            return TSTATUS_DONE;
        }

        // ehh
        ai->nav.validate_path();

        // FIXME Make it handle this weirdness better.
        if (ai->nav.get_path()->size() == 0) {
            cout << "NAV: ERROR CALCULATING PATH <<<<<<<<<<<<<" << endl;
            return TSTATUS_ABORT;
        }

        switch (step) {

        // First we should back up a bit to try to avoid some walls.
        case FN_NEXT_STEP:
        {
            auto aw = new TaskAvoidWalls(1);
            ai->queue_task(aw);
            step = FN_TURN_WAIT;
            return TSTATUS_CONTINUE;
        }
        
        // This basically forces us to wait until we've stopped turning before
        // it'll let us interrupt again.
        case FN_TURN_WAIT:
        {
            step = FN_NORMAL;
            return TSTATUS_CONTINUE;
        }

        // Now go towards the next waypoint.
        case FN_NORMAL:
        {
            Vec2i target = ai->nav.get_current_target();
            move_delegate = new TaskMoveTowards(target, 5, -1);
            ai->queue_task(move_delegate);
            return TSTATUS_CONTINUE;
        }

        }

        return TSTATUS_ABORT;
    }

    std::string name() override {
        return "FOLLOW_NAV";
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

    // Update the navigation and display.
    std::vector<Vec2i> chg_cells; // not actually used
    update_nav(robo, &state->nav, chg_cells);
    draw_known_world(robo, state, chg_cells);
    //draw_samples(robo);

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

void robot_thread(Robot* robot) {

    robot_state = new aistate;
    robot_state->active = true;
    //robot_state->queue_task(new TaskGo(M_PI / 2, 2));
    //robot_state->queue_task(new DebugPrintLidarTask());
    //robot_state->queue_task(new TaskExplore());
    //robot_state->queue_task(new TaskDelay(10 * 60 * 1000, true));
    //robot_state->queue_task(new TaskMoveTowards(Vec2i(0, 0), 5, 2));
    robot_state->queue_task(new TaskFollowNav());

    robot->do_stuff();

}

int viz_run() {
    int ysize = VIZ_WIDTH;
    int xsize = VIZ_HEIGHT;

    char c = 0;

    // Open a new window for drawing.
    gfx_open(xsize, ysize, "why isn't this working aaaaa");

    // Set the current drawing colors.
    gfx_color(0, 200, 100);
    gfx_clear_color(0, 0, 0);

    // Draw a triangle on the screen.
    //gfx_line(100, 100, 200, 100);
    //gfx_line(200, 100, 150, 150);
    //gfx_line(150, 150, 100, 100);

    while (1) {
        draw_mutex->lock();
        gfx_lock();
        if (gfx_event_waiting()) {
            // Wait for the user to press a character.
            c = gfx_wait();
        }
        gfx_unlock();
        draw_mutex->unlock();

        // Quit if it is the letter q.
        if (c == 'q') break;

        usleep(40 * 1000);
    }

    return 0;
}

int main(int argc, char* argv[]) {
    cout << "making robot" << endl;

    // Don't worry about this yet.
    srand(0);

    draw_mutex = new std::mutex();

    Robot robot(argc, argv, callback);
    std::thread rthr(robot_thread, &robot);

    return viz_run();
}
