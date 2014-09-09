#include "ros/ros.h"
#include "gui_msg/msg_gui_ros.h"

/**
 * This tutorial demonstrates simple receipt of messages over the ROS system.
 */
void requestCallback(const gui_msg::msg_gui_ros::ConstPtr& msg)
{
  ROS_INFO("I heard: codice [%ld] messaggio [%s]", msg->codice, msg->messaggio.c_str());
}

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("gui_ros_msg", 1000, requestCallback);

  ros::spin();

  return 0;
}
