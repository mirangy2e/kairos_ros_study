#include "ros/ros.h"                          // ROS Default Header File
#include "ros_serial_topic/MsgSerial.h"  // MsgSerial Message File Header. The header file is automatically created when building the package.

// Message callback function. This is a function is called when a topic
// message named 'ros_serial_msg' is received. As an input message,
// the 'MsgSerial' message of the 'ros_serial_topic' package is received.
void msgCallback(const ros_serial_topic::MsgSerial::ConstPtr& msg)
{
  ROS_INFO("recieve msg = %f", msg->temp);        // Prints the 'data' message
}

int main(int argc, char **argv)                         // Node Main Function
{
  ros::init(argc, argv, "serial_topic_subscriber");            // Initializes Node Name

  ros::NodeHandle nh;                                   // Node handle declaration for communication with ROS system

  // Declares subscriber. Create subscriber 'serial_topic_subscriber' using the 'MsgSerial'
  // message file from the 'ros_serial_topic' package. The topic name is
  // 'ros_serial_msg' and the size of the publisher queue is set to 100.
  ros::Subscriber ros_serial_sub = nh.subscribe("ros_serial_msg", 100, msgCallback);

  // A function for calling a callback function, waiting for a message to be
  // received, and executing a callback function when it is received.
  ros::spin();

  return 0;
}
