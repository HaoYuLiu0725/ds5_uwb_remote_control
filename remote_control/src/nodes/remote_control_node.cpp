#include "remote_control/remote_control.h"

using namespace remote_control;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "remote_control_node");
  ros::NodeHandle nh("");
  ros::NodeHandle nh_local("~");

  try
  {
    ROS_INFO("[Remote Control]: Initializing node");
    RemoteControl remote_control(nh, nh_local);
    ros::spin();
  }
  catch (const char* s)
  {
    ROS_FATAL_STREAM("[Remote Control]: " << s);
  }
  catch (...)
  {
    ROS_FATAL_STREAM("[Remote Control]: Unexpected error");
  }

  return 0;
}
