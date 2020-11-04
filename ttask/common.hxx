
#ifndef _COMMON_HXX_
#define _COMMON_HXX_

#include <iostream>
#include <vector>
#include <string>

#include <math.h>
	
using std::cout;
using std::endl;

#include "robot.hh"
#include "nav.hxx"

#define CARD_EAST 0
#define CARD_NORTH
#define CARD_WEST
#define CARD_SOUTH

#define GOAL_X 20.0
#define GOAL_Y 0.0

extern std::string cardinalv[4];

float distf(float, float, float, float);

/**
 * Computes angle difference, accounting for the weirdness with the angle
 * system.
 *
 * Positive means turn CCW, negative means turn CW.
 */
float angle_diff(float from, float to);

#define calc_cardinal_hdg(card) ((card - 1) * M_PI_2)
#define cardinal_name(card) (cardinalv[card])
int find_nearest_cardinal(float hdg);

uint64_t get_now_time();

/**
 * Computes an average of multiple angle readings, accounting for the fact that
 * it's over a circle.
 *
 * Expects angles to be in the weird coordinate format, returns an angle in the
 * weird coordinate format.
 */
float angle_avg(float* angv, size_t angc);

/**
 * value, min, max
 */
float clampf(float v, float n, float x);

/**
 * value, dead zone
 *
 * Works on floats and returns an int.
 */ 
int signf2i(float v, float dz);

struct DistSamples {
    float brt_dist;
    float right_dist;
    float frt_dist;
    float fwd_dist;
    float flt_dist;
    float left_dist;
    float blt_dist;

    DistSamples(Robot*);
};

#define TSTATE_NEW 0
#define TSTATE_ACTIVE 1
#define TSTATE_DONE 2

extern std::string statenamev[3];

#define TSTATUS_DONE 0
#define TSTATUS_CONTINUE -1
#define TSTATUS_ABORT -2
#define TSTATUS_INTERRUPT -3

class RoboTask;
struct aistate;

struct TaskEntry {
    // 0 = new, 1 = active, 2 = done
    int state;
    RoboTask* task;
    std::string* comment;
    
    TaskEntry(RoboTask*);
    ~TaskEntry();
};

class RoboTask {
public:
    // Generic counter for whatever purpose a task might need.
    int step = 0;
    
    virtual ~RoboTask();

    // Called before we poll this task for the first time.
    // TODO Decide what we want to do with this?
    virtual void init(Robot* robo, aistate* state);
    
    // Called when this task isn't the active task, can be used to cancel
    // running tasks.  Won't be called if we haven't called init() on the task
    // yet.
    virtual int poll_inactive(Robot* robo, aistate* state);

    // Called each tick when the task is active.
    virtual int poll(Robot* robo, aistate* state);
    
    // Returns the name, for printing log messages and such.
    virtual std::string name();
};

struct aistate {
    
    // We do some tasks involving finding lower tasks.
    std::vector<TaskEntry*> task_stack;

    // Tasks to queue.
    std::vector<TaskEntry*> fresh_tasks;

    // Fancy shit for shit.
    NavController nav;

    // Flag for if we should even be polling our tasks.
    bool active;

    bool has_pending_tasks();

    // Adds a task to the queue.  Doesn't immediately put it on the task stack
    // though, that has to come later.
    void queue_task(RoboTask* task);
    
    // Adds a task like above, but also includes a comment.
    void queue_task(RoboTask* task, std::string comment);

    // TODO Remove this, replace with more sophisticated world tracking system.
    std::vector<float> hdg_samples;

    void add_heading_sample(float ang);
    float calc_avg_heading();
    void clear_heading_samples();

    aistate();

};

#endif

