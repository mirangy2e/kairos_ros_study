#include "ros/ros.h"                            // ROS Default Header File
#include "ros_topic_sensorsim/MsgSensor.h"    // MsgTutorial Message File Header. The header file is automatically created when building the package.
#include <time.h>

int main(int argc, char **argv)                 // Node Main Function
{
  ros::init(argc, argv, "ros_topic_sensorsim_publisher");     // Initializes Node Name
  ros::NodeHandle nh;                           // Node handle declaration for communication with ROS system

  // Declare publisher, create publisher 'ros_tutorial_pub' using the 'MsgTutorial'
  // message file from the 'ros_tutorials_topic' package. The topic name is
  // 'ros_tutorial_msg' and the size of the publisher queue is set to 100.
  ros::Publisher ros_topic_sensorsim_pub = nh.advertise<ros_topic_sensorsim::MsgSensor>("ros_topic_sensorsim_msg", 100);

  // Set the loop period. '10' refers to 10 Hz and the main loop repeats at 0.1 second intervals
  ros::Rate loop_rate(10);

  ros_topic_sensorsim::MsgSensor msg;     // Declares message 'msg' in 'MsgTutorial' message file format
  
  srand(time(NULL));

  while (ros::ok())
  {
    msg.temp  = (float)(rand() % 40);                      // Save the the 'count' value in the data of 'msg'

    ROS_INFO("send msg = %f", msg.temp);        // Prints the 'data' message

    ros_topic_sensorsim_pub.publish(msg);          // Publishes 'msg' message

    loop_rate.sleep();                      // Goes to sleep according to the loop rate defined above.

   }

  return 0;
}
