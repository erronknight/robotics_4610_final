#ifndef SEARCH_TASK_HH_
#define SEARCH_TASK_HH_

#include <memory>
#include <type_traits>
#include "MotionMacros.hh"
#include "ScanTask.hh"
#include "State.hh"
#include "Task.hh"
#include "robot.hh"

// Searches in a given Direction.
class SearchTask : public Task {
 private:
  // The Direction we are searching.
  MM::Direction direction_;

  // "done" flag.
  bool done_ = false;
  int last_scan_x_;
  bool detect_turn_node();

 public:
  // Constructor.
  SearchTask(Robot* robot, MM::Direction direction)
      : Task(robot), direction_(direction) {
    last_scan_x_ = g_state->pos_x;
  }

  // Destructor.
  ~SearchTask() {}

  // Override.
  bool is_done();

  // Override.
  void finish();

  // Override.
  void proceed();
};

bool SearchTask::is_done() { return done_; }

void SearchTask::finish() {
  std::cout << "Obstacle detected. Switching directions." << std::endl;
  robot_->set_vel(0, 0);

  MM::Direction next_dir = MM::Direction::kLeft;
  if (direction_ == MM::Direction::kLeft) {
    next_dir = MM::Direction::kRight;
  }

  // Scan, then start searching in the opposite direction.
  g_state->add_task(std::make_shared<SearchTask>(robot_, next_dir));
  g_state->add_task(std::make_shared<ScanTask>(robot_, MM::Turn::kRight));

  done_ = true;
}

void SearchTask::proceed() {
  static constexpr auto kVelocity = 4;
  const auto kTargetTheta = static_cast<int>(direction_);
  const auto kRobotTheta = Utils::rads_to_degrees(robot_->pos_t);
  const auto kDoorWidth = 30;

  if (robot_->at_goal()) {
    robot_->done();
    finish();
  }

  // Moved enough distance. Scan for a doorway.
  if (g_state->pos_x % kDoorWidth == 0 && g_state->pos_x != last_scan_x_) {
    auto scan_dir = MM::Turn::kRight;
    scan_dir = direction_ == MM::Direction::kLeft ? MM::Turn::kRight : scan_dir;

    // Ensure no duplicate scan tasks.
    if (!dynamic_cast<ScanTask*>(g_state->tasks.back().get())) {
      g_state->add_task(std::make_shared<ScanTask>(robot_, scan_dir));
    }
  }

  // Adjust the robot's direction if needed.
  if (MM::adjust_direction(robot_, direction_)) {
    return;
  }

  // We hit a soccer_wall so we've searched as far as possible.
  if (MM::is_facing(robot_, direction_, 3) && robot_->range < 1.0) {
    finish();
  }

  // Move forward
  robot_->set_vel(3.5, 3.5);
  g_state->pos_x += (direction_ == MM::Direction::kRight) ? 1 : -1;
}

bool SearchTask::detect_turn_node() {
  return MM::is_facing(robot_, MM::Direction::kFront) && robot_->range < 2;
}

#endif  // SEARCH_TASK_HH_
