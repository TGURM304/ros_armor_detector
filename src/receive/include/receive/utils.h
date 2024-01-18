#ifndef UTILS_H
#define UTILS_H

double getPro(const cv::RotatedRect &r);
int getAngle(const cv::RotatedRect &r1, const cv::RotatedRect &r2);
void processContours(std::vector< std::vector<cv::Point> > &contours);
std::pair < std::vector<cv::Point>, std::vector<cv::Point> > findArmor(std::vector< std::vector<cv::Point> > &contours, double mn, double mx);
std::pair < std::vector<cv::Point>, std::vector<cv::Point> > findArmorEx(std::vector< std::vector<cv::Point> > &contours, double mn1, double mx1, double mn2, double mx2);

#endif

