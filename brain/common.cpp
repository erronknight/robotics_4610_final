#include "common.hxx"

std::string cardinalv[] = {
    "EAST",
    "NORTH",
    "WEST",
    "SOUTH"
};

Vec2i::Vec2i() : x(0), y(0) {
    // nothing else
}

Vec2i::Vec2i(int x, int y) : x(x), y(y) {
    // nothing else
}

Vec2f Vec2i::to_vec2f() const {
    return Vec2f((float) x, (float) y);
}

Vec2f::Vec2f() : x(0), y(0) {
    // nothing else
}

Vec2f::Vec2f(float x, float y) : x(x), y(y) {
    // nothing else
}

Vec2i Vec2f::to_vec2i() const {
    return Vec2i((int) roundf(x), (int) roundf(y));
}

float distf(float x1, float y1, float x2, float y2) {
    float dx = x2 - x1;
    float dy = y2 - y1;
    return sqrtf(dx * dx + dy * dy);
}

/**
 * Computes angle difference, accounting for the weirdness with the angle
 * system.
 *
 * Positive means turn CCW, negative means turn CW.
 */
float angle_diff(float from, float to) {
    float from_adj = from < 0 ? (2 * M_PI) + from : from;
    float to_adj = to < 0 ? (2 * M_PI) + to : to;
    float raw_delta = fmodf(to_adj - from_adj, 2 * M_PI);

    if (raw_delta < -M_PI) {
        return (2 * M_PI) + raw_delta;
    }

    if (raw_delta > M_PI) {
        return (-2 * M_PI) + raw_delta;
    }

    return raw_delta;
}

uint64_t get_now_time() {
    struct timeval now;
    gettimeofday(&now, NULL);
    return now.tv_sec * 1000 + now.tv_usec / 1000;
}

int find_nearest_cardinal(float hdg) {
    int raw = roundf(hdg / M_PI_2) + 1;
    return (raw + 4) % 4;
}

float angle_avg(float* angv, size_t angc) {
    float sinsum = 0;
    float cossum = 0;

    for (size_t i = 0; i < angc; i++) {
        float ang = angv[i];
        float adj = ang < 0 ? (2 * M_PI) + ang : ang;
        sinsum += sinf(adj);
        cossum += cosf(adj);
    }

    float avg = atan2f(sinsum, cossum);
    return avg > M_PI ? (-2 * M_PI) + avg : avg;
}

float clampf(float v, float n, float x) {
    return fminf(fmaxf(v, n), x);
}

int signf2i(float v, float dz) {
    if (v > dz) {
        return 1;
    }
    if (v < -dz) {
        return -1;
    }
    return 0;
}

DistSamples::DistSamples(Robot* robo) {
/*    // These indexes are always stable.
    brt_dist = robo->ranges[0].range;
    right_dist = robo->ranges[1].range;
    frt_dist = robo->ranges[2].range;
    fwd_dist = robo->ranges[3].range;
    flt_dist = robo->ranges[4].range;
    left_dist = robo->ranges[5].range;
    blt_dist = robo->ranges[6].range;*/
}

bool aistate::has_pending_tasks() {
    return task_stack.size() > 0 || fresh_tasks.size() > 0;;
}

void aistate::queue_task(RoboTask* task) {
    task->preinit(this);
    TaskEntry* te = new TaskEntry(task);
    fresh_tasks.push_back(te);
}

void aistate::queue_task(RoboTask* task, std::string comment) {
    task->preinit(this);
    TaskEntry* te = new TaskEntry(task);
    te->comment = new std::string(comment);
    fresh_tasks.push_back(te);
}

void aistate::add_heading_sample(float ang) {
    hdg_samples.push_back(ang);
}

float aistate::calc_avg_heading() {
    return angle_avg(hdg_samples.data(), hdg_samples.size());
}

void aistate::clear_heading_samples() {
    hdg_samples.clear();
}

std::string statenamev[] = {
    "NEW",
    "ACTIVE",
    "DONE"
};

TaskEntry::TaskEntry(RoboTask* tsk) {
    state = TSTATE_NEW;
    task = tsk;
    comment = NULL;
}

TaskEntry::~TaskEntry() {
    if (comment != NULL) {
        delete comment;
    }
}

RoboTask::~RoboTask() {
    // nothing!
}

void RoboTask::preinit(aistate* state) {
    ai = state;
}

void RoboTask::init(Robot* robo) {
    //return TSTATUS_CONTINUE;
}

int RoboTask::poll_inactive(Robot* robo) {
    return TSTATUS_CONTINUE;
}

int RoboTask::poll(Robot* robo) {
    return TSTATUS_DONE;
}

std::string RoboTask::name() {
    return "(unnamed)";
}

aistate::aistate() {
    // nothing yet, I guess
}

