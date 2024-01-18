#include "CameraApi.h" //相机SDK的API头文件

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <ros/ros.h>
//image_transport是专门用于图像订阅与发布的
#include<image_transport/image_transport.h>
//cv_bridge作为ROS图像与openCV图像的数据转换桥梁
#include<cv_bridge/cv_bridge.h>
#include "std_msgs/Int32.h"
#include "std_msgs/Int16MultiArray.h"

//2个点 中心点的x，y
int center_pipul_point[50];
std_msgs::Int16MultiArray pipul_point_msg;
ros::Publisher point_;
void point_callback(const std_msgs::Int16MultiArray::ConstPtr& msg);
int main(int argc,char** argv)
{
    ros::init(argc, argv, "center_point_");
    std::cout<<"center_point_ 节点启动成功"<<std::endl;
    ros::NodeHandle nh_;
    
    //订阅点的话题
    ros::Subscriber match_up=nh_.subscribe("/center_point", 10, &point_callback);
    //发布最终的中心点
    point_=nh_.advertise<std_msgs::Int16MultiArray>("/lastest_center_point",10);
    ros::spin();
    return 0;
}

void point_callback(const std_msgs::Int16MultiArray::ConstPtr& msg)
{
	// Modified by tiny_fish. 20240114
    int sz = msg->data.size();
    if(!sz) return;
    for(int i = 0; i < sz; i++){
    	center_pipul_point[i] = msg->data[i];
    }

	double dis = 1e18;
	int last[2] = {0, 0};
	
	
    for(int i=0; i < sz; i += 2){
		double d = sqrt(pow(center_pipul_point[i]-320, 2)+pow(center_pipul_point[i+1]-240, 2));
        if(d < dis){
			last[0] = center_pipul_point[i];
			last[1] = center_pipul_point[i+1];
            dis = d;
        }
    }

	// ?	
    center_pipul_point[0] = last[0];
    center_pipul_point[1] = last[1];

    pipul_point_msg.data.clear();
	std::cout << last[0] << ", " << last[1] << std::endl;
    pipul_point_msg.data.push_back(center_pipul_point[0]);
    pipul_point_msg.data.push_back(center_pipul_point[1]);
    point_.publish(pipul_point_msg);
}
