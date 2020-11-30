#ifndef _TASKRT_HXX_
#define _TASKRT_HXX_

#include "robot.hh"
#include "common.hxx"

void print_stack_trace(aistate* state);
void do_abort(Robot* robo, aistate* ai);
void do_poll_tasks(Robot* robo, aistate* ai);

#endif
