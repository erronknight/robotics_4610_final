#include <iostream>
#include <string>
#include <vector>

#include <time.h>
#include <stdlib.h>
#include <sys/types.h>
#include <unistd.h>

#include "robot.hh"

using std::cout;
using std::endl;
using std::string;
using std::vector;

using namespace gazebo;
using namespace gazebo::transport;

const double GOAL_X = 20.0;
const double GOAL_Y = 0.0;

double
clamp(double xmin, double xx, double xmax)
{
    if (xx < xmin) return xmin;
    if (xx > xmax) return xmax;
    return xx;
}

void
nudge(int* ee)
{
    int roll = rand() % 6;
    if (roll == 0) {
        *ee += 1;
    }
    if (roll == 1) {
        *ee -= 1;
    }
    if (*ee > 10) {
        *ee = 10;
    }
    if (*ee < -10) {
        *ee = -10;
    }
}

float
degrade(float xx, int ee)
{
    /*
    float aa = 1.0f + 0.05 * float(ee);
    return aa * xx;
    */
    return 1.05*xx + 0.2*ee;
}

Robot::Robot(int argc, char* argv[], void (*cb)(Robot*))
    : on_update(cb), task_done(false), stamp(0.0f),
      err_x(5), err_y(0), err_l(1), err_r(0)
{
    srand(getpid()^time(0));

    client::setup(argc, argv);
    node = NodePtr(new Node());
    node->Init();

    vel_pub = node->Advertise<msgs::Any>("~/tankbot0/vel_cmd", 50);
    vel_pub->WaitForConnection();

    cout << "advertise on " << vel_pub->GetTopic() << endl;

    scan_sub = node->Subscribe(
        string("~/tankbot0/tankbot/lazstar/link/laser/scan"),
        &Robot::on_scan,
        this,
        false
    );

    pose_sub = node->Subscribe(
        string("~/tankbot0/pose"),
        &Robot::on_pose,
        this,
        false
    );

    cout << "robot created" << endl;
}

Robot::~Robot()
{
    client::shutdown();
    cout << "robot destroyed" << endl;
}

void
Robot::do_stuff()
{
    while (!task_done) {
        gazebo::common::Time::MSleep(10);

        if (this->at_goal()) {
            this->set_vel(0.0, 0.0);
            this->done();
        }
    }

    gazebo::common::Time::MSleep(100);
}

void
Robot::update()
{
    nudge(&(this->err_x));
    nudge(&(this->err_y));
    nudge(&(this->err_l));
    nudge(&(this->err_r));

    this->pos_x = degrade(raw_x, err_x);
    this->pos_y = degrade(raw_y, err_y);
    this->pos_t = raw_t; // no error on heading position, not worth it anymore

    this->on_update(this);
}

bool
Robot::at_goal()
{
    double dx = GOAL_X - this->raw_x;
    double dy = GOAL_Y - this->raw_y;
    return (abs(dx) < 0.75 && abs(dy) < 0.75);
}


void
Robot::done()
{
    this->task_done = true;
}

void
Robot::set_vel(double lvel, double rvel)
{
    lvel = clamp(-5, degrade(lvel, err_l), 5);
    rvel = clamp(-5, degrade(rvel, err_r), 5);
    int xx = 128 + int(lvel * 25.0);
    int yy = 128 + int(rvel * 25.0);
    auto msg = msgs::ConvertAny(xx * 256 + yy);
    this->vel_pub->Publish(msg);
}

void
Robot::on_scan(ConstLaserScanStampedPtr &msg)
{
    gazebo::common::Time tt = gazebo::msgs::Convert(msg->time());
    this->stamp = tt.Float();

    msgs::LaserScan scan = msg->scan();
    auto xs = scan.ranges();

    this->ranges.clear();
    for (long ii = 0; ii < xs.size(); ++ii) {
        double range = xs[ii];
        double theta = scan.angle_min() + ii*scan.angle_step();
        this->ranges.push_back(LaserHit(range, theta));
    }

    this->update();
}

void
Robot::on_pose(ConstPoseStampedPtr &msg)
{
    gazebo::common::Time tt = gazebo::msgs::Convert(msg->time());
    this->stamp = tt.Float();

    auto pos = msg->pose().position();
    this->raw_x = pos.x();
    this->raw_y = pos.y();

    auto rot = msg->pose().orientation();
    ignition::math::Quaternion<double> qrot(rot.w(), rot.x(), rot.y(), rot.z());
    this->raw_t = qrot.Yaw();

    this->update();
}
