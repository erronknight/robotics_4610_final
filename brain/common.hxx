
#ifndef _COMMON_HXX_
#define _COMMON_HXX_

#include <iostream>
#include <vector>
#include <string>

#include <math.h>

using std::cout;
using std::endl;

#include "robot.hh"

#define CARD_EAST 0
#define CARD_NORTH
#define CARD_WEST
#define CARD_SOUTH

#define GOAL_X 20.0
#define GOAL_Y 0.0

extern std::string cardinalv[4];

struct Vec2i;
struct Vec2f;

struct Vec2i {
    int x, y;

public:
    Vec2i();
    Vec2i(int x, int y);

    Vec2f to_vec2f() const;

    bool operator==(const Vec2i& rhs) const {
        return x == rhs.x && y == rhs.y;
    }
};

struct Vec2f {
    float x, y;

public:
    Vec2f();
    Vec2f(float x, float y);

    Vec2i to_vec2i() const;

    bool operator==(const Vec2f& rhs) const {
        return x == rhs.x && y == rhs.y;
    }
    
    Vec2f operator +(const Vec2f& rhs) const {
        return Vec2f(x + rhs.x, y + rhs.y);
    }
};

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

extern std::string statenamev[3];

class RoboTask;
struct aistate;

#define TSTATE_NEW 0
#define TSTATE_ACTIVE 1
#define TSTATE_DONE 2

struct TaskEntry {
    // 0 = new, 1 = active, 2 = done
    int state;
    RoboTask* task;
    std::string* comment;

    TaskEntry(RoboTask*);
    ~TaskEntry();
};

#define TSTATUS_DONE 0
#define TSTATUS_CONTINUE -1
#define TSTATUS_ABORT -2
#define TSTATUS_INTERRUPT -3

class RoboTask {
public:
    // Generic counter for whatever purpose a task might need.
    int step = 0;

    // Overall AI of the robot we're a part of.
    aistate* ai;

    virtual ~RoboTask();

    // Used internally to load AI state.
    virtual void preinit(aistate* state);

    // Called before we poll this task for the first time.
    // TODO Decide what we want to do with this?
    virtual void init(Robot* robo);

    // Called when this task isn't the active task, can be used to cancel
    // running tasks.  Won't be called if we haven't called init() on the task
    // yet.
    virtual int poll_inactive(Robot* robo);

    // Called each tick when the task is active.
    virtual int poll(Robot* robo);

    // Returns the name, for printing log messages and such.
    virtual std::string name();
};

struct aistate {

    // We do some tasks involving finding lower tasks.
    std::vector<TaskEntry*> task_stack;

    // Tasks to queue.
    std::vector<TaskEntry*> fresh_tasks;

    // Flag for if we should even be polling our tasks.
    bool active;

    bool has_pending_tasks();

    // Adds a task to the queue.  Doesn't immediately put it on the task stack
    // though, that has to come later.
    void queue_task(RoboTask* task);

    // Adds a task like above, but also includes a comment.
    void queue_task(RoboTask* task, std::string comment);

    aistate();

};

#endif

