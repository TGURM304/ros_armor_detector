/*
 	Written by tiny_fish.
	@Date: 2024-01-15
	@Description: Useful tools.
*/

// TODO: Kalman Filter, ... optimization algorithm...

#include <vector>
#include <cmath>
#include <algorithm>

#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <ros/ros.h>

#include "receive/utils.h"

// define for std::pair <int, int>
#define fi first
#define se second

double getPro(const cv::RotatedRect &r){
	// Get the proportion (\in (0, 1]) of input rectangle.
	double w = r.size.width, h = r.size.height;
	return std::min(w, h) / std::max(w, h);	
}

int getAngle(const cv::RotatedRect &r1, const cv::RotatedRect &r2){
	// Get the angle between two rectangles.
	CvPoint2D32f p1[4], p2[4];
	cvBoxPoints(r1, p1); cvBoxPoints(r2, p2);
	std::vector <cv::Point2f> v1, v2;

	v1.push_back(p1[0]), v1.push_back(r1.size.width >= r1.size.height ? p1[3] : p1[1]);
	v2.push_back(p2[0]), v2.push_back(r2.size.width >= r2.size.height ? p2[3] : p2[1]);

	cv::Point dt[] = {v1[1] - v1[0], v2[1] - v2[0]};
	double angle[] = {
		atan2(dt[0].x, dt[0].y) * 180 / acos(-1),
		atan2(dt[1].x, dt[1].y) * 180 / acos(-1)
	};

	int ret;
	if(angle[0] * angle[1] >= 0){
		ret = (int)std::abs(angle[0] - angle[1]);	
	} else {
		ret = (int)(std::abs(angle[0]) + std::abs(angle[1]));
		if(ret > 180) ret = 360 - ret;
	}
	ret %= 180;
	if(ret > 90) ret = 90 - ret;
	return ret;
}

void processContours(std::vector< std::vector<cv::Point> > &contours){
	// filter
	std::vector< std::vector<cv::Point> > tmp;
	for(auto item : contours){
		auto rect = cv::boundingRect(item);
		double x = rect.x, y = rect.y, w = rect.width, h = rect.height;
		// TODO: optimization
		if(cv::contourArea(item) < 50 || w * 1.0 / h > 2){ // area restrict  warninggggggggggggggggg
			// erase
			// contours.erase(item);  // errrrrrrrrrr
			tmp.push_back(item);
		}
	}
	for(auto item : tmp) contours.erase(std::find(contours.begin(), contours.end(), item)); // why ?
	if(contours.empty()) return;
	// sort by X
	std::sort(contours.begin(), contours.end(), 
	[&](std::vector<cv::Point> x, std::vector<cv::Point> y){
		return cv::minAreaRect(x).center.x < cv::minAreaRect(y).center.x;
	});
}

std::pair < std::vector<cv::Point>, std::vector<cv::Point> > findArmor(
	std::vector< std::vector<cv::Point> > &contours, 
	double mn, double mx
){
	// Find the most similar armor in contours.
	if(contours.empty()) return {};
	int sz = contours.size();
	// std::cout << "sz = " << sz << std::endl;
	std::pair < std::vector<cv::Point>, std::vector<cv::Point> > ret = {{}, {}};
	int mnAngle = 100, mxAngleLimit = 6;
	cv::RotatedRect rect[2];
	
	for(int i = 0; i < sz; i++){
		rect[0] = cv::minAreaRect(contours[i]);
		for(int j = i+1; j < sz; j++){
			// printf("%d %d\n", i, j);
			rect[1] = cv::minAreaRect(contours[j]);
			int angle = getAngle(rect[0], rect[1]);
			if(angle > mxAngleLimit || angle > mnAngle) continue;
			std::vector <cv::Point> both;
			both.insert(both.end(), contours[i].begin(), contours[i].end());
			both.insert(both.end(), contours[j].begin(), contours[j].end());
			double bothp = getPro(cv::minAreaRect(both));
			if(bothp < mn || bothp > mx) continue;
			// TODO: more filters
			mnAngle = angle, ret.fi = contours[i], ret.se = contours[j];
		}
	}

	// erase begin
	// contours.erase(contours.begin() + ret.fi);
	// contours.erase(contours.begin() + ret.se - 1);
	
	if(ret.fi.size()){
		contours.erase(std::find(contours.begin(), contours.end(), ret.fi));
		contours.erase(std::find(contours.begin(), contours.end(), ret.se));
	}
	
	// erase end
	return ret;
}

std::pair < std::vector<cv::Point>, std::vector<cv::Point> > findArmorEx(
	std::vector< std::vector<cv::Point> > &contours, 
	double mn1, double mx1,
	double mn2, double mx2
){
	// Find the most similar armor in contours.
	if(contours.empty()) return {};
	int sz = contours.size();
	// std::cout << "sz = " << sz << std::endl;
	std::pair < std::vector<cv::Point>, std::vector<cv::Point> > ret = {{}, {}};
	int mnAngle = 100, mxAngleLimit = 6;
	cv::RotatedRect rect[2];
	
	for(int i = 0; i < sz; i++){
		rect[0] = cv::minAreaRect(contours[i]);
		for(int j = i+1; j < sz; j++){
			// printf("%d %d\n", i, j);
			rect[1] = cv::minAreaRect(contours[j]);
			int angle = getAngle(rect[0], rect[1]);
			if(angle > mxAngleLimit || angle > mnAngle) continue;
			std::vector <cv::Point> both;
			both.insert(both.end(), contours[i].begin(), contours[i].end());
			both.insert(both.end(), contours[j].begin(), contours[j].end());
			double bothp = getPro(cv::minAreaRect(both));
			if((bothp < mn1 || bothp > mx1) && (bothp < mn2 || bothp > mx2)) continue;
			// TODO: more filters
			mnAngle = angle, ret.fi = contours[i], ret.se = contours[j];
		}
	}

	// erase begin
	// contours.erase(contours.begin() + ret.fi);
	// contours.erase(contours.begin() + ret.se - 1);
	
	if(ret.fi.size()){
		contours.erase(std::find(contours.begin(), contours.end(), ret.fi));
		contours.erase(std::find(contours.begin(), contours.end(), ret.se));
	}
	
	// erase end
	return ret;
}
