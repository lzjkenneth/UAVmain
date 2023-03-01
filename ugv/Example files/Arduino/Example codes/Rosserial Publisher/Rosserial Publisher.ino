#include <ros.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

std_msgs::String str_msg;

ros::Publisher chatter("chatter", &str_msg); //declare that we want to publish string message to the topic "chatter" with a class named "chatter"

char hello[13] = "hello world!";

void setup()
{
  nh.initNode();
  nh.advertise(chatter);
}

void loop()
{
  str_msg.data = hello;
  chatter.publish( &str_msg ); //publish the string message
  nh.spinOnce();
  delay(1000);
}
