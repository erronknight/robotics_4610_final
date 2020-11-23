
#include <iostream>

#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>

using std::string;
using std::vector;
using namespace gazebo;
using physics::ModelPtr;
using physics::JointControllerPtr;
using ignition::math::Pose3d;
using common::Time;

class TankControlPlugin : public ModelPlugin {
public:
    physics::ModelPtr model;
    vector<string> drives_l;
    vector<string> drives_r;
    vector<string> drives_arm_l;
    vector<string> drives_arm_r;
    vector<string> drives_kick;

    float kicker_length = 0.25;

    transport::NodePtr node;

    transport::SubscriberPtr arm_sub;
    transport::SubscriberPtr kick_sub;
    transport::SubscriberPtr vel_sub;
    transport::SubscriberPtr stat_sub;

    transport::PublisherPtr  pose_pub;

    TankControlPlugin() {}

    void
    SetVel(double lvel, double rvel) {
        auto jc = model->GetJointController();
        for (auto name : this->drives_l) {
            jc->SetVelocityTarget(name, lvel);
            std::cerr << "svtl " << name << lvel << std::endl;
        }
        for (auto name : this->drives_r) {
            jc->SetVelocityTarget(name, rvel);
            std::cerr << "svtr " << name << rvel << std::endl;
        }
    }

    void
    SetArmAngle(double angle) {
        auto jc = model->GetJointController();
        for (auto name : this->drives_arm_r) {
            jc->SetPositionTarget(name, -angle);
            std::cerr << "spgr " << name << -angle << std::endl;
        }
        for (auto name : this->drives_arm_l) {
            jc->SetPositionTarget(name, angle);
            std::cerr << "spgl " << name << angle << std::endl;
        }
    }

    void
    SetKickPos(double pos) {
      auto jc = model->GetJointController();
      for (auto name : this->drives_kick) {
        jc->SetPositionTarget(name, pos);
        std::cerr << "spk " << name << pos << std::endl;
      }
    }

    void
    SetVelPID(string name) {
        auto pid = common::PID(0.15, 0, 0);
        auto jc = model->GetJointController();
        jc->SetVelocityPID(name, pid);
    }

    void
    SetArmPID(string name) {
        auto pid = common::PID(1.0, 0.1, 1.0);
        auto jc = model->GetJointController();
        jc->SetPositionPID(name, pid);
    }

    void
    SetKickPID(string name) {
        auto pid = common::PID(25.0, 1.0, 1.0);
        auto jc = model->GetJointController();
        jc->SetPositionPID(name, pid);
    }

    virtual void
    Load(physics::ModelPtr model, sdf::ElementPtr sdf) {
        this->model = model;

        if (model->GetJointCount() == 0) {
            std::cerr << "bad model loaded" << std::endl;
        }

        auto model_name = this->model->GetName();
        auto world_name = this->model->GetWorld()->Name();

        std::cerr << "hello from ControlPlugin" << std::endl;
        std::cerr << "world: " << world_name << std::endl;
        std::cerr << "model: " << model_name << std::endl;

        for (auto joint : model->GetJoints()) {
            auto name = joint->GetName();
            auto sname = joint->GetScopedName();

            if (name == std::string("tankbot::wheel_fl_drive") ||
                name == std::string("tankbot::wheel_rl_drive"))
            {
                this->drives_l.push_back(sname);
                this->SetVelPID(sname);
            }

            if (name == std::string("tankbot::wheel_fr_drive") ||
                name == std::string("tankbot::wheel_rr_drive"))
            {
                this->drives_r.push_back(sname);
                this->SetVelPID(sname);
            }

            if (name == std::string("tankbot::left_shoulder_joint")) {
                this->drives_arm_l.push_back(sname);
                this->SetArmPID(sname);
            }

            if (name == std::string("tankbot::right_shoulder_joint")) {
                this->drives_arm_r.push_back(sname);
                this->SetArmPID(sname);
            }

            if (name == std::string("tankbot::kicker_joint")) {
                this->drives_kick.push_back(sname);
                this->SetKickPID(sname);
            }

            std::cerr << "joint: " << joint->GetName() << std::endl;
        }

        this->SetVel(0.0, 0.0);
        this->SetArmAngle(0);
        this->SetKickPos(0);

        this->node = transport::NodePtr(new transport::Node());
        this->node->Init(world_name);

        string vel_cmd_topic = "~/" + model_name + "/vel_cmd";
        this->vel_sub = this->node->Subscribe(vel_cmd_topic, &TankControlPlugin::OnVelCmd, this);
        std::cerr << "Subscribed vel_cmd: " << this->vel_sub->GetTopic() << std::endl;

        string arm_cmd_topic = "~/" + model_name + "/arm_cmd";
        this->arm_sub = this->node->Subscribe(arm_cmd_topic, &TankControlPlugin::OnArmCmd, this);
        std::cerr << "Subscribed arm_cmd: " << this->arm_sub->GetTopic() << std::endl;

        string kick_cmd_topic = "~/" + model_name + "/kick_cmd";
        this->kick_sub = this->node->Subscribe(kick_cmd_topic, &TankControlPlugin::OnKickCmd, this);
        std::cerr << "Subscribed kick_cmd: " << this->kick_sub->GetTopic() << std::endl;

        string stats_topic = "~/world_stats";
        this->stat_sub = this->node->Subscribe(stats_topic, &TankControlPlugin::OnStats, this);
        std::cerr << "Subscribed world_stats: " << this->stat_sub->GetTopic() << std::endl;

        string pose_topic = "~/" + model_name + "/pose";
        this->pose_pub = this->node->Advertise<msgs::PoseStamped>(pose_topic, 50);
        std::cerr << "Advertised pose" << std::endl;

        std::cerr << "tank control loaded" << std::endl;
    }

    void
    OnVelCmd(ConstAnyPtr &msg) {
        int raw = msg->int_value();
        int xx = raw / 256 - 128;
        int yy = raw % 256 - 128;
        double lvel = 3 * (xx / 25.0);
        double rvel = 3 * (yy / 25.0);

        std::cerr << "Got vel cmd: " << lvel << "," << rvel << std::endl;

        this->SetVel(lvel, rvel);
    }

    void
    OnArmCmd(ConstAnyPtr &msg) {
        int raw = msg->int_value();
        double rad_angle = float(raw) / 128.0 - 1.0;
        std::cerr << "Got pos cmd: " << raw << " " << rad_angle << std::endl;
        this->SetArmAngle(rad_angle * 3.14159265);
    }

    void
    OnKickCmd(ConstAnyPtr &msg) {
        int raw = msg->int_value();
        double kick_pos = (float(raw) / 256.0) * kicker_length;
        std::cerr << "Got kick cmd: " << raw << " " << kick_pos << std::endl;
        this->SetKickPos(kick_pos);
    }

    msgs::PoseStamped
    make_pose_msg(Time time, Pose3d pose) {
        msgs::PoseStamped ps;
        // time (Time)
        auto time_msg = ps.mutable_time();
        time_msg->set_sec(time.sec);
        time_msg->set_nsec(time.nsec);

        // pose (Pose)
        auto pose_msg = ps.mutable_pose();

        // // position (Vector3d)
        auto pos = pose.Pos();
        auto pos_msg = pose_msg->mutable_position();
        pos_msg->set_x(pos.X());
        pos_msg->set_y(pos.Y());
        pos_msg->set_z(pos.Z());

        // // orientation (Quaternion)
        auto ori = pose.Rot();
        auto ori_msg = pose_msg->mutable_orientation();
        ori_msg->set_x(ori.X());
        ori_msg->set_y(ori.Y());
        ori_msg->set_z(ori.Z());
        ori_msg->set_w(ori.W());

        return ps;
    }

    void
    OnStats(ConstAnyPtr &_msg) {
        auto pose = this->model->WorldPose();
        auto time = Time::GetWallTime();
        auto msg = make_pose_msg(time, pose);
        this->pose_pub->Publish(msg);
    }
};

GZ_REGISTER_MODEL_PLUGIN(TankControlPlugin)
