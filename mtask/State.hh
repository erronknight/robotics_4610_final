#ifndef STATE_HH
#define STATE_HH

#include <iostream>
#include <memory>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

#include "Task.hh"

typedef std::vector<std::shared_ptr<Task>> TaskQueue;

class State final {
 public:
  //// State Variables ////
  TaskQueue tasks;

  // Constructor.
  State() { this->tasks = {}; }

  // Disable cloning.
  State(State &clone) {}

  // Disable assignment.
  void operator=(const State &) {}

  // Signals the state to update itself and the robot.
  void update(Robot *robot) {
    if (tasks.size() == 0) {
      return;
    }
    if (tasks.back()->is_done()) {
      tasks.pop_back();
      return;
    }
    tasks.back()->proceed();
  }

  // Updates the current task.
  void add_task(std::shared_ptr<Task> task) {
    std::cout << "New task pushed to queue." << std::endl;
    if (!task->is_done()) {
      this->tasks.push_back(std::move(task));
    }
  }

  // Returns ``true`` if a Task is in progress.
  bool is_busy() { return (this->tasks.size() == 0) ? false : true; }
};

std::shared_ptr<State> g_state = std::make_shared<State>();

#endif  // STATE_HH
