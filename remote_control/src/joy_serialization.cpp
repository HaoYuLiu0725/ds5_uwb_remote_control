#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/String.h>
ros::Publisher string_pub;
void joy_callback(sensor_msgs::Joy::ConstPtr msg)
{
  uint32_t serial_size = ros::serialization::serializationLength(*msg);
  boost::shared_ptr<uint8_t[]> buffer(new uint8_t[serial_size]);

  ros::serialization::OStream stream(buffer.get(), serial_size);
  ros::serialization::serialize(stream, *msg);
  std_msgs::String s;
  s.data.assign(reinterpret_cast<char*>(buffer.get()), serial_size);
  string_pub.publish(s);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "joy_serialization");
  ros::NodeHandle nh;
  ros::Subscriber twist_sub = nh.subscribe<sensor_msgs::Joy>("joy", 10, &joy_callback);
  string_pub = nh.advertise<std_msgs::String>("serialized_msg", 10);
  ros::spin();
}
