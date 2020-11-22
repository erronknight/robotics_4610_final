#include <iostream>
#include <string>
#include <vector>

#include "opencv2/core/utility.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"

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

Robot::Robot(int argc, char* argv[], void (*cb)(Robot*))
    : on_update(cb), task_done(false)
{
    client::setup(argc, argv);
    node = NodePtr(new Node());
    node->Init();

    vel_pub = node->Advertise<msgs::Any>("~/tankbot0/vel_cmd");
    vel_pub->WaitForConnection();

    scan_sub = node->Subscribe(
        string("~/tankbot0/tankbot/ultrasonic_sensor/link/sonar/sonar"),
        &Robot::on_scan,
        this,
        false
    );

    frame_sub = node->Subscribe(
        string("~/tankbot0/tankbot/camera_sensor/link/camera/image"),
        &Robot::on_frame,
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

bool
Robot::at_goal()
{
    double dx = GOAL_X - this->pos_x;
    double dy = GOAL_Y - this->pos_y;
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
    auto r_error = lvel * ((rand() % 21) - 10) * 0.01;
    auto l_error = rvel * ((rand() % 21) - 10) * 0.01;

    lvel = clamp(-4, lvel + l_error, 4);
    rvel = clamp(-4, rvel + r_error, 4);

    //cout << "set_vel: " << lvel << "," << rvel << endl;
    int xx = 128 + int(lvel * 25.0);
    int yy = 128 + int(rvel * 25.0);
    auto msg = msgs::ConvertAny(xx * 256 + yy);
    //cout << "send vmsg = " << msg.int_value() << endl;
    this->vel_pub->Publish(msg);
}

void
Robot::on_scan(ConstSonarStampedPtr &msg)
{
    //cout << "Receiving the Sonar message from Sonar Sensor" << endl;
    msgs::Sonar sonar = msg->sonar();
    double hit_range = sonar.range();
    range = hit_range;

    this->on_update(this);
}

void
Robot::on_frame(ConstImageStampedPtr &msg)
{
    msgs::Image image = msg->image();

    char* data = (char*)malloc(image.data().size());
    memcpy(data, image.data().c_str(), image.data().size());
    cv::Mat temp(image.height(), image.width(), CV_8UC3, data);
    cv::Mat temp2 = temp.clone();
    cv::cvtColor(temp, temp2, cv::COLOR_RGBA2BGRA);
    this->frame = temp2.clone();
    assert(this->frame.size().height > 0);
    free(data);

    this->on_update(this);
}

void
Robot::on_pose(ConstPoseStampedPtr &msg)
{
    auto x_error = ((rand() % 21) - 10) * 0.02;
    auto y_error = ((rand() % 21) - 10) * 0.02;

    auto pos = msg->pose().position();
    this->pos_x = pos.x() + x_error;
    this->pos_y = pos.y() + y_error;

    auto rot = msg->pose().orientation();
    ignition::math::Quaternion<double> qrot(rot.w(), rot.x(), rot.y(), rot.z());
    this->pos_t = qrot.Yaw();

    this->on_update(this);
}
