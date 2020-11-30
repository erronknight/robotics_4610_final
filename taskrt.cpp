/*
 * Contains task management runtime implementation.
 */

#include "taskrt.hxx"

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
