#include <ros.h>
#include <std_msgs/Empty.h>

ros::NodeHandle nh;

void messageCb( const std_msgs::Empty& log_msg){
  nh.loginfo("Hello UAVenture"); //print the log message in the terminal
}

ros::Subscriber<std_msgs::Empty> sub("logger", &messageCb ); 

void setup()
{
  nh.initNode();
  nh.subscribe(sub);
}

void loop()
{
  nh.spinOnce();
  delay(1);
}