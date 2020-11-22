#ifndef ROBOT_HH
#define ROBOT_HH

#include <gazebo/gazebo_config.h>
#include <gazebo/transport/transport.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/gazebo_client.hh>

#include <opencv2/core/mat.hpp>

double clamp(double, double, double);

class Robot {
  public:
    void (*on_update)(Robot*);

    float pos_x;
    float pos_y;
    float range;
    float pos_t;
    cv::Mat frame;

    Robot(int argc, char* argv[], void (*cb)(Robot*));
    ~Robot();

    bool at_goal();
    void do_stuff();
    void done();

    void set_vel(double lvel, double rvel);

    void on_scan(ConstSonarStampedPtr &msg);
    void on_frame(ConstImageStampedPtr &msg);
    void on_pose(ConstPoseStampedPtr &msg);

  private:
    bool task_done;

    gazebo::transport::NodePtr node;
    gazebo::transport::PublisherPtr vel_pub;
    gazebo::transport::SubscriberPtr scan_sub;
    gazebo::transport::SubscriberPtr frame_sub;
    gazebo::transport::SubscriberPtr pose_sub;
};

#endif
