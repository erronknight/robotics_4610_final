# Task System

## Overview

The core primitives is the `RoboTask` class.  The most important function that
needs to be overridden in `RoboTask` is `int poll(Robot* robo)`.  This is called
repeatedly by the harness whenever the task is the "current" task.  The `int`
return code is elaborated on below.

Task management takes the form of a stack in the `aistate` struct.  The common
paradigm is to have a main task that is added to the stack before the AI starts
and that will queue any other tasks it needs to perform its responsibilities.

When a task is queued, it has its `ai` field populated with the `aistate` it is
a part of, so it can queue other tasks just by saying `ai->queue_task(...)`.

## Lifecycle

(See `do_poll_tasks(Robot*, aistate*)` in `taskrt.cpp` for implementation.)

There's 3 states that a task can have:

* `TSTATE_NEW` - newly added, unticked
* `TSTATE_ACTIVE` - running, to be polled when current
* `TSTATE_DONE` - finished, won't be polled, set to be cleaned up

When newly added, a task enters the `NEW` state.  If the AI decides that the
task is the current task it will check if it's set as `NEW` and call the
`void init(Robot*)` function on it in order to initialize it.  By default this
does nothing, and implementing it is optional.  Then it proceeds to invoke
`poll` as normal.

There are 4 return codes that `poll` (and `poll_inactive`) can return:

* `TSTATUS_DONE` - marks the task as `DONE`, eventually removing it from the queue
* `TSTATUS_CONTINUE` - does nothing so that the task will be polled again eventually
* `TSTATUS_ABORT` - aborts the AI, dumping the task stack like a stack trace with debug information
* `TSTATUS_INTERRUPT` - interrupts tasks higher on the stack (see below)

Before invoking the current task, the AI invokes `int poll_inactive(Robot*)` on
all of the tasks deeper on the stack, starting from index 0 to the task before
the current task.  It skips tasks that are not marked as `ACTIVE`.  This can be
used for tasks to interrupt tasks that they scheduled, by returning with
`TSTATUS_INTERRUPT`.  I used this to implement switching to a new path when
implementing my A* AI.  This function does nothing by default.  Try to only
interact with `Robot*` in a read-only way, as modifying it will confuse the
current task.  I should probably have made this a `const` pointer or something.

If `poll_inactive` returns `TSTATUS_DONE`, then the higher tasks will continue
executing, but the task won't be polled any longer and will clean up when
appropriate.  If `poll` returns `TSTATUS_INTERRUPT` then the AI prints and error
and continues as if `TSTATUS_CONTINUE` were called.

## Implementation

There's a `step` field in `RoboTask` that can be used as a common way of keeping
track of what a task is currently trying to accomplish.  I commonly made
`#define`s of the names of the steps.  It's useful to include this as it's
included in the task trace.  But **don't** overuse it.

**Most state that a task needs should be implemented as fields on the task.**
Let's take `TaskMoveTowards` as one example:

(header class def, `tasklib.hxx`)

```cpp
class TaskMoveTowards : public RoboTask {
    Vec2i target_pos;
    float base_vel;
    float end_dist;
    TaskGo* move_delegate;

public:
    TaskMoveTowards(Vec2i start_tgt, float vel, float dist);
    int poll_inactive(Robot* robo) override;
    int poll(Robot* robo) override;
    std::string name() override;
    void update_target(Vec2i* new_tgt);
    void update_target(Vec2f* new_tgt);
};
```

(source class impl, `tasklib.cpp`)

```cpp
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
    std::string buf = "MOVE";
    //buf << target_pos.x << ", " << target_pos.y << ")";
    return buf;
}

void TaskMoveTowards::update_target(Vec2i new_tgt) {
    target_pos = new_tgt;
}
```

Here we can see one use of the `step` counter and some cross-task communication.

The actual "going towards a target" is implemented by `TaskGo`, which tries to
move towards a heading while turning.  You can update its target by calling
`update_heading` on it, and its PID loop is used to adjust the steering as
necessary.  We don't have to worry much about the ownership of these tasks as we
know that the `aistate` owns the tasks and will delete them at the right times
as long as you don't do something really weird with them.

In that task we also override the destructor in order to `delete` the `PID` that
it uses.  If I used `std::unique_ptr` instead this wouldn't be necessary, but
I'm a C programmer and that's complicated.  The parameters to the PID could use
more tuning if someone is good at that and wants to work on it.

Another way to do cross-task communcation is to just pass a pointer from a
parent to the child in its constructor and have the child populate the target of
the pointer as necessary.  Tasks won't be moved after being created so even
pointers to members is safe.  If you really want to then using `std::shared_ptr`
to avoid any issues isn't wrong, but you shouldn't need to worry about it.

Honestly for the final I don't think we're going to have to do much cross-task
communcation, so don't worry about it too much.

## Debugging

The `std::string name()` function is called on a task when printing debug
messages about task switches and when printing the task trace.  It can be
computed on the fly to include whatever information you want about the task,
such as the current destination.  This is only one line, so be careful.

The convention is that names are ALL_CAPS and extra info is included in parens.
I was trying to make `MOVE` be `MOVE(13, 37)` to include the target x and y but
I got lazy when my first attempt didn't work.

When queuing a task, there's the default `queue_task(RoboTask*)` function on
`aistate`, but there's a version `queue_task(RoboTask*, std::string)` that will
include a comment in the task trace when aborting that can be used to attribute
why a task was put on the stack.

**Really Hot Tip:** One thing that you can do when testing a specific task
behavior is to comment out adding the main task to the queue when setting up the
AI and add the task for the behavior you're testing.  This also gives you the
opportunity to try various parameters read in from CLI arguments instead of
having to edit the source and recompile

## Misc

Feel free to add fields to `aistate` and to the callback if you can't find a
reasonable way to fit something into the task system.  When I implemented A* I
did it using a `NavController` class that knew nothing about tasks and just
"ran in the background" via calls in the callback function.  There was another
task that followed the pathway that the nav controller was generating and would
queue a `TaskMoveTowards` and update it to follow the path, and interrupt it
when a new path was decided.  It would have worked well if there just wasn't so
much noise.

The `tasklib.{hxx,cpp}` is supposed to be for general-purpose tasks.  Anything
that is specific to a particular bot behavior should go in the `main_*.cpp` for
the bot.  If there's something specific to this assignment but not specific to a
bot behavior, then you might want to make another source file for it and include
it separately.

There's some code organization that probably could be more logical.  It's pretty
ok as-is, it's just unfortunate that some things can't really be split apart any
further than they are.

## Organization

In case it's unclear, I figured I should include this so people know what
they're looking at:

* `common.*` - top-level generic functions
* `tasklib.*` - generic, general-purpose movement tasks and stuff, think of it like a stdlib but for tasks
* `taskrt.*` - task runtime, like internal task management code
* `pid.*` - simply PID loop implementation

I also had `nav.*` in my A* assignment for doing A* stuff.
