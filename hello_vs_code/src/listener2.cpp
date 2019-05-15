#include "ros/ros.h"
#include "std_msgs/String.h"

void talkerCallback(const std_msgs::String::ConstPtr& rosMsg)
{
    ROS_INFO("I heard: [%s]", rosMsg->data.c_str());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "listener2");
    ros::NodeHandle hanlde;

    ros::Subscriber talker_sub = hanlde.subscribe("/talker", 1000, talkerCallback);
    ros::spin();
    
    return 0;
}