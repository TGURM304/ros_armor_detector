/*
    Written by tiny_fish.
    @date: 2024/01/18
    @description: 图像预处理，接受 /origin_image 下的图像，处理后发布到 /processed_image 话题中。
*/

#include <cstdio>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

image_transport::Publisher processed_pub;

// cv::Scalar rRange[2] = {cv::Scalar(0, 9, 197), cv::Scalar(236, 251, 245)};
cv::Scalar rRange[2] = {cv::Scalar(0, 9, 197), cv::Scalar(255, 255, 255)};
cv::Scalar bRange[2] = {cv::Scalar(7, 0, 178), cv::Scalar(203, 255, 255)};

cv::Mat getMask(cv::Mat img, cv::Scalar *rng){
	// img: origin image
	// rng: colour range
	cv::Mat res;
	cv::cvtColor(img, res, cv::COLOR_BGR2HSV);
	cv::medianBlur(res, res, 5);
	cv::inRange(res, rng[0], rng[1], res);
	return res;
}

void process(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    } catch(cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat origin_image = cv_ptr->image;
    cv::Mat mask = getMask(origin_image, rRange);
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
	cv::erode(mask, mask, kernel);
    cv::dilate(mask, mask, kernel);
    cv::dilate(mask, mask, kernel);
    processed_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", mask).toImageMsg());
}

int main(int argc, char **argv){
	ros::init(argc, argv, "image_process");
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_(nh_);

	// subscribe & advertise
	auto origin_sub = it_.subscribe("/origin_image", 5, process);
	processed_pub = it_.advertise("/processed_image", 5);

	ROS_INFO("Node 'image_process' initialized.");
    ros::spin();
	return 0;
}
