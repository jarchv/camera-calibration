#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
void CenterRecognition(cv::Mat src);

//using namespace cv;
//using namespace std;


double max_thresh  = 255;

cv::Mat frame;
cv::Mat frameResult;
cv::Mat gray;
cv::Mat normImg;
cv::Mat bin;    
cv::Mat rot;
cv::Mat dst;
cv::Mat dst2;
cv::Mat imgT;
cv::Mat imgT2;
cv::Mat frameLab2;
cv::Mat channels[3];
cv::Mat lab_image;
cv::Mat lab;
cv::Mat diff;
cv::Mat prev_frame;

cv::Point minMBB = cv::Point(115,80);
cv::Point maxMBB = cv::Point(434,340);

int countFrame      = 0;
int cb              = 0;
int thresh          = 100;
int corners_prev[4] = {0, 0, 0, 0};

float time_avr;
float minc = 1000.0;

clock_t begin_time;

std::vector<cv::Point> RpdCnts;

bool isIncluded(std::vector<cv::Point> X, cv::Point Pt);
void findCenters(cv::Mat frameResult);
void thresholdIntegral(cv::Mat &inputMat, cv::Mat &outputMat);
void getSquareImage(cv::Mat img, cv::Mat dst, int size);
void showImages(const std::string& window_name, int rows, int cols, int size, std::initializer_list<cv::Mat*> images, int pad = 1);

