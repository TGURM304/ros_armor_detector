/*
	@description: 从工业相机获取图像，发布到 origin_image 话题中。
*/
#include <cstdio>
#include <iostream>
#include <CameraApi.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <unistd.h> // sleep

struct CAM{
	int                     iCameraCounts = 1;
    int                     iStatus = -1;
    tSdkCameraDevInfo       tCameraEnumList;
    int                     hCamera;
    tSdkCameraCapbility     tCapability;
    tSdkFrameHead           sFrameInfo;
    BYTE*                   pbyBuffer;
    int                     iDisplayFrames = 10000;
    IplImage 				*iplImage = NULL;
    int                     channel = 3;

	unsigned char			*rgbBuffer;

	int init(){
		iCameraCounts = 1; // 检测的相机数量（若多相机需要更改这里的数值）
		// 下面函数执行完之后 iCameraCounts 变成实际检测到的相机数量
		CameraSdkInit(1); CameraEnumerateDevice(&tCameraEnumList, &iCameraCounts);
		if(!iCameraCounts){
			printf("%d %d %d\n", iCameraCounts, iStatus, hCamera);
			puts("Err: Camera not found.");
			return -1;
		}
		if(iStatus = CameraInit(&tCameraEnumList, -1, -1, &hCamera)){
			printf("Err: Camera init fault. (%d, %d)\n", iStatus, hCamera);
			return -1;		
		}
		std::cout << "hCamera = " << hCamera << std::endl;
		// Get Capability
		CameraGetCapability(hCamera, &tCapability);
		// Init rgbBuffer
		rgbBuffer = (unsigned char *)malloc(
			tCapability.sResolutionRange.iHeightMax * tCapability.sResolutionRange.iWidthMax * 3
		);
		CameraPlay(hCamera);
		if(tCapability.sIspCapacity.bMonoSensor){
			channel = 1;
			CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_MONO8);	
		} else {
			channel = 3;
			CameraSetIspOutFormat(hCamera, CAMERA_MEDIA_TYPE_BGR8);	
		}
		return 0;
	}

	int rngInit(int times = 0){
		// 循环尝试 times 次，默认为无穷。
		if(!times){
			while(init()){
				ROS_INFO("Waiting for camera...");
				usleep(500000);	// 500ms
			}
			return 0;
		} else {
			while(times--){
				ROS_INFO("Waiting for camera... (%d)", times+1);
				if(!init()) return 0;
				usleep(500000);	// 500ms
			}
			return -1;
		}
	}

	void uninit(){
		CameraUnInit(hCamera);
		free(rgbBuffer);
	}

	cv::Mat getImage(bool reverse = true){
		cv::Mat res;
		if(CameraGetImageBuffer(hCamera, &sFrameInfo, &pbyBuffer, 3000) == CAMERA_STATUS_SUCCESS){
			CameraImageProcess(hCamera, pbyBuffer, rgbBuffer, &sFrameInfo);
			res = cv::Mat(
				// Note: cvSize isn't a member of (namespace) cv.
				cvSize(sFrameInfo.iWidth, sFrameInfo.iHeight),
				sFrameInfo.uiMediaType == CAMERA_MEDIA_TYPE_MONO8 ? CV_8UC1 : CV_8UC3,
				rgbBuffer			
			);
			CameraReleaseImageBuffer(hCamera, pbyBuffer);
			// resize the origin image
			cv::resize(res, res, {640, 480});
			if(reverse){
				// reverse the origin image.
				// cv::flip(res, res, 0);
				// cv::flip(res, res, 1);
				cv::flip(res, res, -1);
			}
			// 如果图像为空就填充成纯黑，避免后面 opencv 爆掉。
			if(res.empty()){ res = cv::Mat::zeros(640, 480, CV_8UC3); }
			return res;
		} else {
			// 视为相机断开连接，重新走连接流程
			// 防止其他节点因所传图像为空而崩溃
			uninit();
			rngInit();
			return cv::Mat::zeros(640, 480, CV_8UC3);
		}
		
	}
}cam;

cv::Scalar rRange[2] = {cv::Scalar(0, 9, 197), cv::Scalar(236, 251, 245)};
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

int main(int argc, char **argv){
	// Modified by tiny_fish. 20240118
	ros::init(argc, argv, "capture_image");
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_(nh_);

	// advertise
	auto origin_pub = it_.advertise("/origin_image", 5);

	// init camera
	if(cam.rngInit(5)) return 0;
	
	cv::Mat origin_image;
	
	ROS_INFO("Node 'capture_image' initialized.");

	while(1){
		// get origin image from camera (reversed)
		origin_image = cam.getImage();
		// publish origin image to "/capture_image"
		// 注意这里是 bgr，不是 rgb ！！！！
		// 不然红蓝通道会反过来。
		origin_pub.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", origin_image).toImageMsg());
	}
	
	// uninit camera
	cam.uninit();

	return 0;
}
