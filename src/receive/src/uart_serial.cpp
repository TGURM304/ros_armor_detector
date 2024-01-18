// Modified by tiny_fish. 20240114
#include <iostream>
#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/Int16MultiArray.h>

#include <cstdio>
#include <algorithm>

serial::Serial sp;

static uint8_t data[10];
static char sdata[50];

int openSerial(){
	sp.setPort("/dev/ttyTHS2");
	sp.setBaudrate(115200);
	serial::Timeout t = serial::Timeout::simpleTimeout(100);
	sp.setTimeout(t);
	try{
		sp.open();	
	} catch(serial::IOException &e) {
		ROS_ERROR("Cannot open serial.\n%s", e.what());
		return -1;	
	}
	if(sp.isOpen()){
		ROS_INFO("Serial opened successfully.");
	} else return -1;
	return 0;
}

void closeSerial(){
	sp.close();
}

void uartCallback(const std_msgs::Int16MultiArray::ConstPtr &msg){
	data[0] = 0xff;
	data[1] = 0xfe;
	data[2] = msg->data[0];
	data[3] = msg->data[0] < 0 ? 0xfb : 0xfa;
	data[4] = msg->data[1];
	data[5] = msg->data[4] < 0 ? 0xfb : 0xfa;
	data[6] = 0xfe;
	data[7] = 0xff;
	sp.write(data, 10);
	ROS_INFO("Sent data: %d, %d", msg->data[0], msg->data[1]);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "uart_serial");
	ros::NodeHandle node;
	ROS_INFO("Node 'uart_serial' initialized.");
	if(openSerial()) return 0;
	ros::Subscriber s = node.subscribe("/yaw_and_pitch", 10, uartCallback);
	ros::spin();
	closeSerial();		
	return 0;
}
