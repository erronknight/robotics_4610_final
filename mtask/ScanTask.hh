#ifndef SCAN_TASK_HH_
#define SCAN_TASK_HH_

#include "MotionMacros.hh"
#include "State.hh"
#include "Task.hh"
#include "ChangeRowTask.hh"
#include "robot.hh"

// Scans for a TurnNode in a circle around the Robot.
class ScanTask : public Task {
 private:
  bool done_ = false;
  float start_theta_;
  int turns_ = 0;
  MM::Turn turn_;

  // Returns true if a node was detected, and adds it to State.
  bool detect_turn_node();

 public:
  // Constructor.
  ScanTask(Robot* robot, MM::Turn turn) : Task(robot), turn_(turn) {
    start_theta_ = Utils::rads_to_degrees(robot_->pos_t);
  }

  // Destructor.
  ~ScanTask() {}

  // Override.
  bool is_done();

  // Override.
  void finish();

  // Override.
  void proceed();
};

bool ScanTask::is_done() { return done_; }

void ScanTask::finish() {
  done_ = true;
  std::cout << "finished Scanning" << std::endl;
}

void ScanTask::proceed() {
  const auto kRobotTheta = Utils::rads_to_degrees(robot_->pos_t);

  if (Utils::in_range(kRobotTheta, start_theta_ - 6, start_theta_ + 6)) {
    if (turns_ > 30 || MM::is_facing(robot_, MM::Direction::kFront, 3)) {
      finish();
    }
  }

  auto found_node = detect_turn_node();
  if (found_node) {
    finish();
  }

  MM::turn(robot_, turn_, 1.6);
  ++turns_;
}

bool ScanTask::detect_turn_node() {
  static constexpr auto kDoorDirection = MM::Direction::kFront;
  bool detected = false;
  if (MM::is_facing(robot_, kDoorDirection, 20)) {
    // Try to detect a door if we're facing front.
    detected = (robot_->range > 1);
    if (detected) {
      std::cout << "Moving up..." << std::endl;

      if (!dynamic_cast<ChangeRowTask*>(g_state->tasks.back().get())) {
        g_state->add_task(std::make_shared<ChangeRowTask>(robot_, kDoorDirection));
      }
      g_state->add_node();
      return detected;
    }
    finish();
  }
  return detected;
}
#endif  // SCAN_TASK_HH_
