#include "ros/ros.h"
#include "std_msgs/String.h"
#include <iostream>

int main(int argc, char **argv)
{

    ros::init(argc, argv, "talker");
    ros::NodeHandle hanlde;

    ros::Publisher talker_pub = hanlde.advertise<std_msgs::String>("/talker", 1000);
    ros::Rate loop_rate(10);
    int count = 0;

    while(ros::ok())
    {
        std_msgs::String rosMsg;
        std::stringstream ss;

        ss << "Hello Ros World [" << count << "]";
        rosMsg.data = ss.str();

        ROS_INFO("%s", rosMsg.data.c_str());
        talker_pub.publish(rosMsg);

        ros::spinOnce(); 
        loop_rate.sleep();

        ++count;
    }

    return 0;
}