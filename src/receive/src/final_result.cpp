// Modified by tiny_fish. 20240114
#include <cstdio>
#include <iostream>

#include <cmath>

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <std_msgs/Int32.h>
#include <std_msgs/Int16MultiArray.h>

ros::Publisher result;
std_msgs::Int16MultiArray msgs;

void process(const std_msgs::Int16MultiArray::ConstPtr &msg){
	msgs.data = msg->data;
	result.publish(msgs);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "final_result");
	ros::NodeHandle node;
	ROS_INFO("Node 'final_result' initialized.");
	ros::Subscriber match_up = node.subscribe("/lastest_center_point", 10, &process);
	result = node.advertise<std_msgs::Int16MultiArray>("/result", 10);
	ros::spin();
	return 0;
}
