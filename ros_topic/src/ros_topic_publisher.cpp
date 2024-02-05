#include "ros/ros.h"                            // ROS Default Header File
#include "ros_topic/Message1.h"    // MsgTutorial Message File Header. The header file is automatically created when building the package.

int main(int argc, char **argv)                 // Node Main Function
{
  ros::init(argc, argv, "ros_topic_publisher");     // Initializes Node Name
  ros::NodeHandle nh;                           // Node handle declaration for communication with ROS system

  // Declare publisher, create publisher 'ros_topic_publisher' using the 'Message1'
  // message file from the 'ros_topic' package. The topic name is
  // 'ros_topic_msg' and the size of the publisher queue is set to 100.
  ros::Publisher ros_topic_pub = nh.advertise<ros_topic::Message1>("ros_topic_msg", 100);

  // Set the loop period. '10' refers to 10 Hz and the main loop repeats at 0.1 second intervals
  ros::Rate loop_rate(10);

  ros_topic::Message1 msg;     // Declares message 'msg' in 'Message1' message file format
  int count = 0;                            // Variable to be used in message

  while (ros::ok())
  {
    msg.stamp = ros::Time::now();           // Save current time in the stamp of 'msg'
    msg.data  = count;                      // Save the the 'count' value in the data of 'msg'

    ROS_INFO("send msg = %d", msg.stamp.sec);   // Prints the 'stamp.sec' message
    ROS_INFO("send msg = %d", msg.stamp.nsec);  // Prints the 'stamp.nsec' message
    ROS_INFO("send msg = %d", msg.data);        // Prints the 'data' message

    ros_topic_pub.publish(msg);          // Publishes 'msg' message

    loop_rate.sleep();                      // Goes to sleep according to the loop rate defined above.

    ++count;                                // Increase count variable by one
  }

  return 0;
}
