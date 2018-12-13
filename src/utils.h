#ifndef UTILS_H
#define UTILS_H

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>

bool isIncluded(std::vector<cv::Point> X, cv::Point Pt);
cv::Mat findCenters(cv::Mat frame, cv::Mat gray, cv::Mat& bin, cv::Mat& contours,int& countFrame,std::vector<cv::Point>& RpdCnts);
void thresholdIntegral(cv::Mat &inputMat, cv::Mat &outputMat);
void getSquareImage(cv::Mat img, cv::Mat dst, int size);
void showImages(const std::string& window_name, int rows, int cols, int size, std::initializer_list<cv::Mat*> images, int pad = 1);

void Mat2Mat(cv::Mat& src, cv::Mat& dst, int x0, int y0);
#endif