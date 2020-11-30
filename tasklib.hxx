
#ifndef _TASKLIB_HXX_
#define _TASKLIB_HXX_

#include "common.hxx"
#include "pid.hxx"

void calc_target_heading(Vec2f origin, Vec2f tgt, float &dx, float &dy, float &tgt_hdg);

/**
 * Task that just waits some duration and returns.  Useful for traveling some
 * distance in a straight line if you don't need to worry about adjusting.
 */
class TaskDelay : public RoboTask {
    uint64_t end_time;
    bool stop;

public:
    TaskDelay(uint64_t ms, bool st);
    void init(Robot* robo) override;
    int poll(Robot* robo) override;
    std::string name() override;
};

class TaskTurnTo : public RoboTask {
    float target_hdg;
    float measured_hdg;
    float ok_thresh;

public:
    TaskTurnTo(float th);
    TaskTurnTo(float th, float okt);
    int poll(Robot* robo) override;
    std::string name() override;
};

class TaskGo : public RoboTask {
    float target_hdg;
    float base_vel;
    PID* pidloop;
    uint64_t lasttime;

public:
    TaskGo(float th, float bv);
    ~TaskGo() override;
    int poll(Robot* robo) override;
    std::string name() override;
    void update_heading(float nhdg);
    void stop();
};

class TaskGoDur : public RoboTask {
    float base_vel;
    float heading;
    uint64_t end_time;

public:
    TaskGoDur(float bv, float hdg, uint64_t et);
    void init(Robot* robo) override;
    int poll(Robot* robo) override;
    std::string name() override;
};

class TaskMoveTowards : public RoboTask {
    Vec2f target_pos;
    float base_vel;
    float end_dist;
    TaskGo* move_delegate;

public:
    TaskMoveTowards(Vec2i start_tgt, float vel, float dist);
    TaskMoveTowards(Vec2f start_tgt, float vel, float dist);
    int poll_inactive(Robot* robo) override;
    int poll(Robot* robo) override;
    std::string name() override;
    void update_target(Vec2i new_tgt);
    void update_target(Vec2f new_tgt);
};

//class DebugPrintPosTask;
//class DebugPrintLidarTask;

#endif
