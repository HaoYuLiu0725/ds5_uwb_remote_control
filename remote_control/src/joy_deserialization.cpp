#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <nlink_parser/LinktrackNodeframe0.h>

ros::Publisher joy_pub;
sensor_msgs::Joy joy;

void node_frame_callback(nlink_parser::LinktrackNodeframe0::ConstPtr msg)
{
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
  ros::serialization::Serializer<sensor_msgs::Joy>::read(stream, joy);

  joy_pub.publish(joy);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joy_deserialization");
  for(int i = 0; i < 6; i++){
    joy.axes.push_back(0);
  }
  for(int i = 0; i < 18; i++){
    joy.buttons.push_back(0);
  }
  ros::NodeHandle nh;
  ros::Subscriber string_sub = nh.subscribe<nlink_parser::LinktrackNodeframe0>("node_frame", 100, &node_frame_callback);
  joy_pub = nh.advertise<sensor_msgs::Joy>("ds5_joy", 100);
  ros::spin();
}
