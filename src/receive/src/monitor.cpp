/*
    Written by tiny_fish.
    @date: 2024/01/18
    @description: 订阅图像并显示出来，后续可结合主从机使用，便于调试。
*/

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>

#define IMAGE_CALLBACK_FUNC(TITLE) \
    cv_bridge::CvImagePtr cv_ptr; \
    try{ \
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); \
    } catch(cv_bridge::Exception &e) { \
        ROS_ERROR("cv_bridge exception: %s", e.what()); \
        return; \
    } \
    cv::imshow(TITLE, cv_ptr->image);\
    cv::waitKey(1);

void originCallback(auto msg){ IMAGE_CALLBACK_FUNC("Origin") }
void resultCallback(auto msg){ IMAGE_CALLBACK_FUNC("Result") }

int main(int argc, char **argv){
    ros::init(argc, argv, "image_monitor");
    ros::NodeHandle node;
    ROS_INFO("Node 'image_monitor' initialized. ");
    image_transport::ImageTransport img(node);
    auto cam_sub = img.subscribe("/origin_image", 5, originCallback);
    auto res_sub = img.subscribe("/result_image", 5, resultCallback);
    ros::spin();
    return 0;
}