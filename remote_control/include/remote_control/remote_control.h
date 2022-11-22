#pragma once

#include <cmath>
#include <random>

#include <ros/ros.h>

#include <std_srvs/Empty.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Point.h>
#include <std_msgs/Bool.h>

namespace remote_control
{
class RemoteControl
{
public:
    RemoteControl(ros::NodeHandle& nh, ros::NodeHandle& nh_local);

private:
    void initialize()
    {
        std_srvs::Empty empt;
        updateParams(empt.request, empt.response);
    }

    bool updateParams(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res);
    void joyCallback(const sensor_msgs::Joy::ConstPtr& ptr);
    void timerCallback(const ros::TimerEvent& e);

    void updateTwist();
    void updatePoint(const ros::TimerEvent& e);
    void updateBool();

    void publish();

    /* ros node */
    ros::NodeHandle nh_;
    ros::NodeHandle nh_local_;
    ros::ServiceServer params_srv_;
    ros::Timer timer_;

    /* ros inter-node */
    ros::Subscriber joy_sub_;
    ros::Publisher twist_pub_;
    ros::Publisher point_pub_;
    ros::Publisher suck_pub_;

    sensor_msgs::Joy input_joy_;
    geometry_msgs::Twist output_twist_;
    geometry_msgs::Point output_point_;
    std_msgs::Bool output_suck_;

    ros::Time last_time_;
    ros::Duration timeout_;

    /* ros param */
    bool p_active_;

    double p_frequency_;
    double p_MAX_linear_speed_;     // [m/s]
    double p_MAX_angular_speed_;    // [rad/s]
    double p_init_arm_x;
    double p_init_arm_y;
    double p_init_arm_z;
    double p_arm_MAX_XYspeed_;      // [mm/s]
    double p_arm_MAX_Zspeed_;       // [mm/s]

    std::string p_joy_topic_;
    std::string p_twist_topic_;
    std::string p_point_topic_;
    std::string p_suck_topic_;
};
}  // namespace remote_control
