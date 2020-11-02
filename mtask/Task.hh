#ifndef TASK_HH_
#define TASK_HH_
#include "robot.hh"

/** Interface for defining a Robot task. */
class Task {
 protected:
  Robot* robot_;

 public:
  /** Constructor. */
  Task(Robot* robot) : robot_(robot) {}

  /** Destructor. */
  virtual ~Task() {}

  /** Controls the Robot as defined by this Task. */
  virtual void proceed() = 0;

  /** Finishes this Task */
  virtual void finish() = 0;

  /** Returns true when this Task is done. */
  virtual bool is_done() = 0;
};

#endif  // TASK_HH_
