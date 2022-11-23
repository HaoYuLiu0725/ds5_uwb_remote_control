#include <remote_control/remote_control.h>

using namespace std;
using namespace remote_control;

RemoteControl::RemoteControl(ros::NodeHandle& nh, ros::NodeHandle& nh_local) : nh_(nh), nh_local_(nh_local)
{
    timer_ = nh_.createTimer(ros::Duration(1.0), &RemoteControl::timerCallback, this, false, false);
    initialize();
}

bool RemoteControl::updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res)
{
    bool get_param_ok = true;
    bool prev_active = p_active_;

    /* get param */
    get_param_ok = nh_local_.param<bool>("active", p_active_, true);

    get_param_ok = nh_local_.param<double>("frequency", p_frequency_, 50);
    get_param_ok = nh_local_.param<double>("MAX_linear_speed", p_MAX_linear_speed_, 0.8);   // [m/s]
    get_param_ok = nh_local_.param<double>("MAX_angular_speed", p_MAX_angular_speed_, 1.0); // [rad/s]
    get_param_ok = nh_local_.param<double>("init_arm_x", p_init_arm_x, 128.0);              // [mm]
    get_param_ok = nh_local_.param<double>("init_arm_y", p_init_arm_y, 17.0);               // [mm]
    get_param_ok = nh_local_.param<double>("init_arm_z", p_init_arm_z, 10.0);               // [mm]
    get_param_ok = nh_local_.param<double>("arm_MAX_XYspeed", p_arm_MAX_XYspeed_, 100.0);   // [mm/s]
    get_param_ok = nh_local_.param<double>("arm_MAX_Zspeed", p_arm_MAX_Zspeed_, 100.0);     // [mm/s]
    get_param_ok = nh_local_.param<double>("X_max", p_X_max_, 516.0);                       // [mm]
    get_param_ok = nh_local_.param<double>("X_min", p_X_min_, -281.0);                      // [mm]
    get_param_ok = nh_local_.param<double>("Y_max", p_Y_max_, 516.0);                       // [mm]
    get_param_ok = nh_local_.param<double>("Y_min", p_Y_min_, -516.0);                      // [mm]
    get_param_ok = nh_local_.param<double>("Z_max", p_Z_max_, 119.0);                       // [mm]
    get_param_ok = nh_local_.param<double>("Z_min", p_Z_min_, -58.0);                       // [mm]

    double timeout;
    get_param_ok = nh_local_.param<double>("timeout", timeout, 0.2);
    timeout_.fromSec(timeout);

    get_param_ok = nh_local_.param<string>("joy_topic", p_joy_topic_, "/ds5_joy");
    get_param_ok = nh_local_.param<string>("twist_topic", p_twist_topic_, "/cmd_vel");
    get_param_ok = nh_local_.param<string>("point_topic", p_point_topic_, "/arm_goal");
    get_param_ok = nh_local_.param<string>("suck_topic", p_suck_topic_, "/suck");

    /* check param */
    if (get_param_ok){
        ROS_INFO_STREAM("[Remote Control]: " << "param set ok");
    }
    else{
        ROS_WARN_STREAM("[Remote Control]: " << "param set fail");
    }

    /* ros node param */
    timer_.setPeriod(ros::Duration(1 / p_frequency_), false);

    if (p_active_ != prev_active)
    {
        if (p_active_)
        {
            joy_sub_ = nh_.subscribe(p_joy_topic_, 10, &RemoteControl::joyCallback, this);
            twist_pub_ = nh_.advertise<geometry_msgs::Twist>(p_twist_topic_, 10);
            point_pub_ = nh_.advertise<geometry_msgs::Point>(p_point_topic_, 10);
            suck_pub_ = nh_.advertise<std_msgs::Bool>(p_suck_topic_, 10);
            timer_.start();
        }
        else
        {
            joy_sub_.shutdown();
            twist_pub_.shutdown();
            point_pub_.shutdown();
            suck_pub_.shutdown();
            timer_.stop();
        }
    }

    /* init state param */
    for(int i = 0; i < 6; i++){
        input_joy_.axes.push_back(0);
    }
    for(int i = 0; i < 18; i++){
        input_joy_.buttons.push_back(0);
    }
    output_twist_.linear.x = 0.0;
    output_twist_.linear.y = 0.0;
    output_twist_.angular.z = 0.0;

    output_point_.x = p_init_arm_x;
    output_point_.y = p_init_arm_y;
    output_point_.z = p_init_arm_z;

    output_suck_.data = false;

    publish();

    return true;
}

void RemoteControl::joyCallback(const sensor_msgs::Joy::ConstPtr& ptr)
{
    input_joy_ = *ptr;
    last_time_ = ros::Time::now();
}

void RemoteControl::timerCallback(const ros::TimerEvent& e)
{
    if(ros::Time::now().toSec() - last_time_.toSec() > timeout_.toSec()){
        return;
    }
    updateTwist();
    updatePoint(e);
    updateBool();
    publish();
}

void RemoteControl::updateTwist()
{
    // Left stick, upward (-1.0 -> 1.0, default ~0.0)
    output_twist_.linear.x = input_joy_.axes[1] * p_MAX_linear_speed_;
    // Left stick, leftward (-1.0 -> 1.0, default ~0.0)
    output_twist_.linear.y = input_joy_.axes[0] * p_MAX_linear_speed_;
    // (L2 - R2), pushDown (0.0 -> 1.0, default = 0)
    output_twist_.angular.z = (input_joy_.axes[4] - input_joy_.axes[5]) * p_MAX_angular_speed_;
}

void RemoteControl::updatePoint(const ros::TimerEvent& e)
{
    double dt = (e.current_expected - e.last_expected).toSec();

    // Right stick, leftward (-1.0 -> 1.0, default ~0.0)
    double dx = - input_joy_.axes[2] * p_arm_MAX_XYspeed_ *  dt;
    // Right stick, upward (-1.0 -> 1.0, default ~0.0)
    double dy = input_joy_.axes[3] * p_arm_MAX_XYspeed_ * dt;
    // (button up - button down)
    double dz = (input_joy_.buttons[13] - input_joy_.buttons[14]) * p_arm_MAX_Zspeed_ * dt;

    output_point_.x += dx;
    output_point_.y += dy;
    output_point_.z += dz;

    output_point_.x = SATURATION(output_point_.x, p_X_min_, p_X_max_);
    output_point_.y = SATURATION(output_point_.y, p_Y_min_, p_Y_max_);
    output_point_.z = SATURATION(output_point_.z, p_Z_min_, p_Z_max_);

}

void RemoteControl::updateBool()
{
    if(input_joy_.buttons[1]) output_suck_.data = true; // button circle
    if(input_joy_.buttons[0]) output_suck_.data = false; // button cross
}

void RemoteControl::publish()
{
    /* twist */
    twist_pub_.publish(output_twist_);

    /* point */
    point_pub_.publish(output_point_);

    /* suck */
    suck_pub_.publish(output_suck_);
}
