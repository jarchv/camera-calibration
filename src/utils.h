#ifndef UTILS_H
#define UTILS_H

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

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
                    std::vector<cv::Point3i>& PointsTracking,
                    int& predictions);

void thresholdIntegral( cv::Mat &inputMat, 
                        cv::Mat &outputMat);
void Mat2Mat(cv::Mat& src, cv::Mat& dst, int x0, int y0);

std::vector<cv::Point> findConcentricCenters( std::vector<cv::RotatedRect> minRect,
                            std::vector<cv::Point> centers,
                            std::vector<float>,
                            int contours_count);


void calcPointPosition(std::vector<cv::Point3f>& corners);

bool Calibration(cv::Size imgSize, 
                cv::Mat& cameraMatrix, 
                cv::Mat& distCoeffs,
                std::vector<std::vector<cv::Point2f>> imagePoints,
                std::vector<cv::Mat>& rvecs,
                std::vector<cv::Mat>& tvecs,
                std::vector<float>& projectErrors,
                double& totalAvgErr);
                
bool SaveParams(cv::Size imgSize, 
                cv::Mat& cameraMatrix, 
                cv::Mat& distCoeffs,
                std::vector<std::vector<cv::Point2f>> imagePoints);
#endif