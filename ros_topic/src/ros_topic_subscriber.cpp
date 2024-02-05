#include "ros/ros.h"                          // ROS Default Header File
#include "ros_topic/Message1.h"  // MsgTutorial Message File Header. The header file is automatically created when building the package.

// Message callback function. This is a function is called when a topic
// message named 'ros_topic_msg' is received. As an input message,
// the 'Message1' message of the 'ros_topic' package is received.
void msgCallback(const ros_topic::Message1::ConstPtr& msg)
{
  ROS_INFO("recieve msg = %d", msg->stamp.sec);   // Prints the 'stamp.sec' message
  ROS_INFO("recieve msg = %d", msg->stamp.nsec);  // Prints the 'stamp.nsec' message
  ROS_INFO("recieve msg = %d", msg->data);        // Prints the 'data' message
}

int main(int argc, char **argv)                         // Node Main Function
{
  ros::init(argc, argv, "ros_topic_subscriber");            // Initializes Node Name

  ros::NodeHandle nh;                                   // Node handle declaration for communication with ROS system

  // Declares subscriber. Create subscriber 'ros_topic_subscriber' using the 'Message1'
  // message file from the 'ros_topic' package. The topic name is
  // 'ros_topic_msg' and the size of the publisher queue is set to 100.
  ros::Subscriber ros_topic_sub = nh.subscribe("ros_topic_msg", 100, msgCallback);

  // A function for calling a callback function, waiting for a message to be
  // received, and executing a callback function when it is received.
  ros::spin();

  return 0;
}
