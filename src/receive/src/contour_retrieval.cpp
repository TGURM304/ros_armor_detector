// TODO: optimization algorithm

#include <cstdio>
#include <iostream>
#include <algorithm>
#include <CameraApi.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Int16MultiArray.h>
#include <image_transport/image_transport.h>

#include <ctime>

#include "receive/utils.h"

// define for std::pair
#define fi first
#define se second

// For FPS Calculating
time_t begin_t, finish_t;

//用于发布装甲板中心点
std_msgs::Int16MultiArray point_msg;
ros::Publisher point_;
image_transport::Publisher result_;

void imageCb(const sensor_msgs::ImageConstPtr& msg);

int main(int argc,char** argv){
    ros::init(argc, argv, "contour_retrieval_");
	ROS_INFO("Node 'contour_retrieval_' initialized.");
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_(nh_);
    image_transport::Subscriber image_sub;
    // image_sub = it_.subscribe("/capture_image/image_manage", 5, &imageCb);
	image_sub = it_.subscribe("/processed_image", 5, &imageCb);
    // 分布点的话题
    point_ = nh_.advertise<std_msgs::Int16MultiArray>("/center_point", 10);
	// 将识别结果的图像发布到话题中，以便 monitor 显示
	result_ = it_.advertise("/result_image", 5);
    ros::spin();
    return 0;
}

double armor_rng[2][3] = {
	{
		0.4,	// short armor proportion
		0.37,	// short armor proportion -> min
		0.6		// short armor proportion -> max
	},
	{
		0.245,	// long armor proportion
		0.22,	// long armor proportion -> min
		0.36	// long armor proportion -> max
	}
};

void imageCb(const sensor_msgs::ImageConstPtr &msg){
	// Modified by tiny_fish. 20240115
	begin_t = clock();

	cv_bridge::CvImagePtr cv_ptr;
	try{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	} catch(cv_bridge::Exception &e) {
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}
	cv::Mat frame = cv_ptr->image, img_canny;
	cv::Canny(frame, img_canny, 100, 100, 3);
	
	std::vector < std::vector<cv::Point> > threshCnts;
	std::vector < cv::Vec4i > hierarchy;
	std::vector <int> filtered, best, bestend;

	std::vector < std::pair< std::vector<cv::Point>, std::vector<cv::Point> > > threshArmor;
	cv::findContours(img_canny, threshCnts, hierarchy, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
	// filter the contours	
	processContours(threshCnts);
	// TODO: optimization
	while(threshCnts.size() >= 2){
		auto res = findArmorEx(threshCnts, armor_rng[0][1], armor_rng[0][2], armor_rng[1][1], armor_rng[1][2]);
		if(res.fi.size()) threshArmor.push_back(res);
		else break;
	}
	for(auto p : threshArmor){
		std::vector <cv::Point> Armor;
		Armor.insert(Armor.end(), p.fi.begin(), p.fi.end());
		Armor.insert(Armor.end(), p.se.begin(), p.se.end());
		cv::Rect r = cv::boundingRect(Armor);
		double x = r.x, y = r.y, w = r.width, h = r.height;
		double center[2] = {x + w/2, y + h/2};
		point_msg.data.push_back(center[0]);
		point_msg.data.push_back(center[1]);
		cv::circle(frame, cv::Point(center[0], center[1]), 7, cv::Scalar(0, 255, 0), 3);
		cv::rectangle(frame, cv::Point(x, y), cv::Point(x+w, y+h), cv::Scalar(255, 0, 0), 3);
		// break; // break for debug
	}

	point_.publish(point_msg);
	point_msg.data.clear();

	finish_t = clock();
	double delta_t = (double)(finish_t - begin_t)/CLOCKS_PER_SEC;
	int fps = 1 / delta_t; char st[10] = ""; sprintf(st, "fps: %d", fps);
	cv::putText(frame, st, cv::Point(75, 50), cv::FONT_HERSHEY_COMPLEX, 0.7, cv::Scalar(0, 0, 255), 2);

	// show (unused)
	// cv::imshow("result", frame);
	// cv::waitKey(1);

	// publish
	result_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", frame).toImageMsg());
}
