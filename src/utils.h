#ifndef UTILS_H
#define UTILS_H

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>

bool isIncluded(std::vector<cv::Point> X,
                cv::Point Pt);

cv::Mat findCenters(cv::Mat frame, 
                    cv::Mat gray, 
                    cv::Mat& bin, 
                    cv::Mat& contours_draw,
                    std::vector<std::vector<cv::Point>>& contours,
                    int& countFrame,
                    std::vector<cv::Point>& RpdCnts,
                    int& predictions);

void thresholdIntegral( cv::Mat &inputMat, 
                        cv::Mat &outputMat);
void Mat2Mat(cv::Mat& src, cv::Mat& dst, int x0, int y0);

#endif