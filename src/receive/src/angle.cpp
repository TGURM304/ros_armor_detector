#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include<iostream>
#include <ros/ros.h>
#include<image_transport/image_transport.h>
#include<geometry_msgs/Twist.h>
#include<cv_bridge/cv_bridge.h>
#include "std_msgs/Int32.h"
#include "std_msgs/Int16MultiArray.h"
std_msgs::Int16MultiArray result_msg;
ros::Publisher result;


    cv::Mat mtx(3, 3, CV_64F, cv::Scalar(0));

   // 定义畸变系数dist    
    cv::Mat dist(1, 5, CV_64F);



    float fx ,fy,cx,cy;

// PinHole_solver 函数
void PinHole_solver(const std_msgs::Int16MultiArray::ConstPtr& msg) {

    float pre_point_x, pre_point_y;
    pre_point_x = msg->data.at(0);
    pre_point_y = msg->data.at(1);

    // 注意这里我们将点从cv::Point2d改为cv::Point2f
    std::vector<cv::Point2f> point = {
      cv::Point2f(static_cast<float>(pre_point_x), static_cast<float>(pre_point_y))
 };
    cv::undistortPoints(point, point, mtx, dist, cv::noArray(), mtx);
    double rxNew = (point[0].x - cx) / fx;
    double ryNew = (point[0].y - cy) / fy;

	std::cout<<fx<<" "<<fy;
    double yaw = std::atan(rxNew) * 180 / CV_PI;
    double pitch = -std::atan(ryNew) * 180 / CV_PI;
    std::cout << "yaw: " << yaw << " pitch: " << pitch << std::endl;
    result_msg.data.clear(); 
    result_msg.data.push_back(yaw);
    result_msg.data.push_back(pitch);
    result.publish(result_msg);
}

int main(int argc, char** argv) {

    mtx.at<double>(0, 0) = 1.08061781e+03; //fx
    mtx.at<double>(0, 1) = 0.00000000e+00;  
    mtx.at<double>(0, 2) = 3.10704118e+02;
    mtx.at<double>(1, 0) = 0.00000000e+00;
    mtx.at<double>(1, 1) = 1.08533116e+03;//fy
    mtx.at<double>(1, 2) = 1.84096856e+02;
    mtx.at<double>(2, 0) = 0.00000000e+00;
    mtx.at<double>(2, 1) = 0.00000000e+00;
    mtx.at<double>(2, 2) = 1.00000000e+00;

    dist.at<double>(0) = -8.31005869e-02;
    dist.at<double>(1) = 1.13120072e+00;
    dist.at<double>(2) = -2.14741910e-02;
    dist.at<double>(3) = 2.99413786e-04;
    dist.at<double>(4) = -4.83596755e+00;


    fx = static_cast<float>(mtx.at<double>(0, 0));
    fy = static_cast<float>(mtx.at<double>(1, 1));
    cx = static_cast<float>(mtx.at<double>(0, 2));
    cy = static_cast<float>(mtx.at<double>(1, 2));

    ros::init(argc, argv, "put_angle");
    std::cout << "Node 'put_angle' initialized." << std::endl; 
    ros::NodeHandle nh;
    ros::Subscriber point_sub = nh.subscribe("/result", 10, &PinHole_solver);
    result = nh.advertise<std_msgs::Int16MultiArray>("/yaw_and_pitch", 10);
    ros::spin();
    return 0;
}
