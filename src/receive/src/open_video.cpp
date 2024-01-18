/*
    Written by tiny_fish.
    @date: 2024/01/18
    @description: 将视频逐帧发布到 origin_image 话题中。用视频代替工业相机，可以用赛时录像更好地测试算法的效果。
    @note: 和 open_camera 不能同时打开！！！
*/
#include <ros/ros.h>
#include <iostream>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <unistd.h> // sleep

// 测试视频路径
char videoPath[] = "test.mp4"; // 可能有的时候需要修改这里的路径，起始是项目的根目录。

int main(int argc, char **argv){
    ros::init(argc, argv, "capture_image");
    ros::NodeHandle node;
    image_transport::ImageTransport img(node);

	// advertise
	auto origin_pub = img.advertise("/origin_image", 5);

    cv::VideoCapture video;
    video.open(videoPath); 
    if(video.isOpened()){
        ROS_INFO("Node 'capture_image' initialized.");
        std::cout << "Width: " << video.get(3) << std::endl;
        std::cout << "Height: " << video.get(4) << std::endl;
        std::cout << "FPS: " << video.get(5) << std::endl;
        std::cout << "FC: " << video.get(7) << std::endl;
    } else {
        ROS_ERROR("Cannot open the test video(PATH: %s)", videoPath);
        return -1;
    }
    cv::Mat origin_image;
    while(1){
        video >> origin_image;
        if(origin_image.empty()) break;
        origin_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", origin_image).toImageMsg());
        usleep(1000000 / video.get(5));
    }
    return 0;
}