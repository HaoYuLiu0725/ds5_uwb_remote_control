#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <nlink_parser/LinktrackNodeframe0.h>

ros::Publisher joy_pub;

void node_frame_callback(nlink_parser::LinktrackNodeframe0::ConstPtr msg)
{
  sensor_msgs::Joy joy;
  uint32_t serial_size = ros::serialization::serializationLength(joy);
  boost::shared_ptr<uint8_t[]> buffer(new uint8_t[serial_size]);
  if (msg->nodes.size())
  {
    for (int i = 0; i < serial_size; i++)
    {
      buffer[i] = msg->nodes[0].data[i];
    }
  }
  ros::serialization::IStream stream(buffer.get(), serial_size);
  ros::serialization::Serializer<geometry_msgs::Twist>::read(stream, joy);

  joy_pub.publish(joy);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joy_deserialization");
  ros::NodeHandle nh;
  ros::Subscriber string_sub = nh.subscribe<nlink_parser::LinktrackNodeframe0>("node_frame", 10, &node_frame_callback);
  joy_pub = nh.advertise<gsensor_msgs::Joy>("ds5_joy", 10);
  ros::spin();
}
