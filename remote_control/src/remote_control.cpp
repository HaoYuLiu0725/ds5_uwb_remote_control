#include "remote_control/remote_control.h"

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

    get_param_ok = nh_local_.param<double>("frequency", p_frequency_, 30);
    get_param_ok = nh_local_.param<double>("MAX_linear_speed", p_MAX_linear_speed_, 0.8);
    get_param_ok = nh_local_.param<double>("MAX_angular_speed", p_MAX_angular_speed_, 1.0);

    double timeout;
    get_param_ok = nh_local_.param<double>("timeout", timeout, 0.2);
    timeout_.fromSec(timeout);

    get_param_ok = nh_local_.param<string>("joy_topic", p_joy_topic_, "/ds5_joy");
    get_param_ok = nh_local_.param<string>("twist_topic", p_twist_topic_, "/cmd_vel");
    get_param_ok = nh_local_.param<string>("point_topic", p_point_topic_, "/arm_goal");
    get_param_ok = nh_local_.param<string>("suck_topic", p_suck_topic_, "/suck");

    /* check param */
    if (get_param_ok)
    {
        ROS_INFO_STREAM("[Odometry]: "
                        << "param set ok");
    }
    else
    {
        ROS_WARN_STREAM("[Odometry]: "
                        << "param set fail");
    }

    /* ros node param */
    timer_.setPeriod(ros::Duration(1 / p_frequency_), false);

    if (p_active_ != prev_active)
    {
        if (p_active_)
        {
        twist_sub_ = nh_.subscribe(p_twist_topic_, 10, &Odometry::twistCallback, this);
        pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(p_pose_topic_, 10);
        odom_pub_ = nh_.advertise<nav_msgs::Odometry>(p_odom_topic_, 10);
        timer_.start();
        }
        else
        {
        twist_sub_.shutdown();
        pose_pub_.shutdown();
        timer_.stop();
        }
    }

    /* init state param */
    output_odom_.pose.pose.position.x = p_init_pose_x;
    output_odom_.pose.pose.position.y = p_init_pose_y;

    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, p_init_pose_yaw);
    output_odom_.pose.pose.orientation = tf2::toMsg(q);

    // clang-format off
                                    //x,   y,   z,   p,   r,   yaw
    output_odom_.pose.covariance = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                    0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

                                    //x,         y,   z,   p,   r,   y
    output_odom_.twist.covariance = {p_cov_vx_, 0.0,       0.0, 0.0, 0.0, 0.0,
                                    0.0,       p_cov_vy_, 0.0, 0.0, 0.0, 0.0,
                                    0.0,       0.0,       0.0, 0.0, 0.0, 0.0,
                                    0.0,       0.0,       0.0, 0.0, 0.0, 0.0,
                                    0.0,       0.0,       0.0, 0.0, 0.0, 0.0,
                                    0.0,       0.0,       0.0, 0.0, 0.0, p_cov_vyaw_};
    // clang-format on

    publish();

    return true;
}

void RemoteControl::twistCallback(const geometry_msgs::Twist::ConstPtr& ptr)
{
  input_twist_ = *ptr;
  last_time_ = ros::Time::now();
}

void RemoteControl::timerCallback(const ros::TimerEvent& e)
{
  if(ros::Time::now().toSec() - last_time_.toSec() > timeout_.toSec()){
    return;
  }
  updateTwist();
  updatePose(e);
  publish();
}

void RemoteControl::updateTwist()
{
  twist_.linear.x = input_twist_.linear.x;
  twist_.linear.y = input_twist_.linear.y;
  twist_.angular.z = input_twist_.angular.z;

  output_odom_.twist.twist = twist_;
}

void RemoteControl::updatePose(const ros::TimerEvent& e)
{
  double dt = (e.current_expected - e.last_expected).toSec();

  double dx = twist_.linear.x * dt;
  double dy = twist_.linear.y * dt;
  double dw = twist_.angular.z * dt;

  double yaw = tf2::getYaw(output_odom_.pose.pose.orientation);

  output_odom_.pose.pose.position.x += (dx * cos(yaw) - dy * sin(yaw));
  output_odom_.pose.pose.position.y += (dx * sin(yaw) + dy * cos(yaw));

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, yaw + dw);
  output_odom_.pose.pose.orientation = tf2::toMsg(q);
}

void RemoteControl::publish()
{
  /* odom */
  ros::Time now = ros::Time::now();
  output_odom_.header.stamp = now;
  output_odom_.header.frame_id = p_fixed_frame_id_;
  output_odom_.child_frame_id = p_target_frame_id_;
  if (p_publish_odom_)
    odom_pub_.publish(output_odom_);

  /* pose */
  static geometry_msgs::PoseWithCovarianceStamped pose;

  pose.header.stamp = now;
  pose.header.frame_id = "map";
  pose.pose = output_odom_.pose;
  if (p_publish_pose_)
    pose_pub_.publish(pose);

  /* tf */
  static geometry_msgs::TransformStamped transform;
  transform.header.stamp = now;
  transform.header.frame_id = p_fixed_frame_id_;
  transform.child_frame_id = p_target_frame_id_;

  transform.transform.translation.x = output_odom_.pose.pose.position.x;
  transform.transform.translation.y = output_odom_.pose.pose.position.y;
  transform.transform.rotation.w = output_odom_.pose.pose.orientation.w;
  transform.transform.rotation.x = output_odom_.pose.pose.orientation.x;
  transform.transform.rotation.y = output_odom_.pose.pose.orientation.y;
  transform.transform.rotation.z = output_odom_.pose.pose.orientation.z;

  if (p_publish_tf_)
    tf2_broadcaster_.sendTransform(transform);
}
