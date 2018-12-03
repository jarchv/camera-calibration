#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <stdlib.h>
//#include <time.h>
#include <unistd.h>
#include <iostream>
using namespace cv;
using namespace std;

int thresh      = 100;
int max_thresh  = 255;

Mat frame;
Mat gray;
Mat normImg;

void   findCenters(int, int);
void transCLAHE(Mat, Mat&);
void  transLineal(Mat, Mat&);
void getMeanValue(Mat,float&,float &);

int main(int argc, char** argv)
{
    
    string filename = argv[1];
    
    VideoCapture cap("files/"+filename);

    if (!cap.isOpened())
    {
        cout << "Failed to open camera." << endl;
        return -1;
    }
    namedWindow("foo",1);

    Mat rot;
    Mat dst;
    Mat dst2;
    Mat imgT;
    Mat imgT2;
    Mat frameLab2;
    float mean;
    float std;
    Mat lab;
    for(;;)
    {
        //usleep(10000);
        cap >> frame;
        if(frame.empty())
            break;

        //if CLAHE
        cv::cvtColor(frame, frameLab2, CV_BGR2Lab);
        
        transCLAHE(frameLab2, imgT2);
        //imshow("clahe",imgT2);
        
        transLineal(imgT2,imgT);
        //imshow("lineal",imgT);
        //cvtColor(imgT, gray, COLOR_BGR2GRAY);
        //Ptr<CLAHE> clahe = createCLAHE();
        //clahe->setClipLimit(4);
        //Mat dst;
        //clahe->apply(imgT,dst);
        //imshow("gray", gray);
        //imshow("dst", dst);
        //imshow("img TranLineal", imgT);
        cvtColor(imgT, gray, COLOR_BGR2GRAY);
        
        // v1 : -30
        // v2 : +10
        
        getMeanValue(gray,mean,std);
        
        double thresh_val = 105;
        std::cout << "thresh_val : " << thresh_val << std::endl; 
        
        //blur( gray, gray, Size(3,3) );
        GaussianBlur(gray, gray, Size(3,3), 0, 0);

        char k = waitKey(1);
        findCenters((int)thresh_val, max_thresh);
        if (k == 27) 
            break;
    }

    cap.release();
}

void findCenters(int thresh, int max_thresh)
{
    Mat threshold_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    threshold( gray, threshold_output, thresh, max_thresh, THRESH_BINARY );
    Mat element = getStructuringElement( MORPH_ELLIPSE,
                                       Size( 3, 3),
                                       Point( 1, 1 ) );
    erode( threshold_output, threshold_output, element );
    //imshow("threshold", threshold_output);

    findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE, Point(0, 0));
    vector<Point> centers(contours.size());
    vector<float> radius(contours.size());

    vector<RotatedRect> minRect( contours.size() );
    vector<RotatedRect> minEllipse( contours.size() );
    int counte = 0;
    for( int i = 0; i < contours.size(); i++ )
    {
        if( contours[i].size() > 6  && contours[i].size() < 150)
        {
            minRect[counte]    = minAreaRect( Mat(contours[i]) );
            minEllipse[counte] = fitEllipse( Mat(contours[i]) );
            counte++;
        }
    }

    Scalar color;
    Point2f rect_points[4];
    int centerx;
    int centery;
    int radiustemp;
    int errormaxradio = 7;
    int errormax = 3; 

    Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );

    for( int i = 0; i< counte; i++ )
    {
        color = Scalar( 255, 255, 0);
        minRect[i].points( rect_points );
        centerx = 0.5*(rect_points[0].x + rect_points[2].x);
        centery = 0.5*(rect_points[0].y + rect_points[2].y);
        centers[i] = Point(centerx,centery); 
        radius[i] =  sqrt((rect_points[2].x - rect_points[0].x)*(rect_points[2].x - rect_points[0].x)+(rect_points[2].y - rect_points[0].y)*(rect_points[2].y - rect_points[0].y));
    }

    for(size_t p = 0;p<counte;p++){
        minRect[p].points( rect_points );
        centerx = 0.5*(rect_points[0].x + rect_points[2].x);
        centery = 0.5*(rect_points[0].y + rect_points[2].y);
        int count = 0;
        radiustemp = sqrt(pow((rect_points[2].x - rect_points[0].x),2)+pow((rect_points[2].y - rect_points[0].y),2));
        for(size_t k = 0;k<counte;k++)
        {
            if(k != p){
                float dist = sqrt((centerx - centers[k].x)*(centerx - centers[k].x) + (centery - centers[k].y)*(centery - centers[k].y));
                if((dist <= errormax) && (abs(radiustemp - radius[k])>errormaxradio))
                {
                    count++;
                }
                if(count > 0){
                    ellipse( frame, minEllipse[p], color, 2, 8);
                    break;
                } 
            }
        }
    }
    imshow("foo", frame);
}

void transCLAHE(Mat inputImg,Mat& imgT)
{
    //Mat ch1, ch2, ch3;
    //inputImg.convertTo(inputImg, CV_32FC1);
    std::vector<cv::Mat> channels(3);
    Mat sum;
    Mat lab_image;
    double min, max;
    split(inputImg, channels);

    //sum = channels[0] + channels[1] + channels[2];

    //minMaxLoc(sum, &min, &max);
    //std::cout <<  max << std::endl;

    //std::cout << "sum : " << sum.size() << std::endl;
    //for (int i = 0; i < 3; i++)
    //{
        //std::cout << "i :" << i <<" channels : " << channels[i].size() << std::endl;
        //channels[i] /= sum;
        //channels[i] *= 255;
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(3);
    cv::Mat dst;
    clahe->apply(channels[0], dst);
    dst.copyTo(channels[0]);
    cv::merge(channels, lab_image);

    cv::cvtColor(lab_image, imgT, CV_Lab2BGR);
            //std::cout << "i :" << i <<" channels : " << channels[i].size() << std::endl;
        //minMaxLoc(channels[i], &min, &max);
        //std::cout <<  max << std::endl;
        //channels[i] = 255 * (channels[i] - min)/ (max - min);
        //channels[i].convertTo(channels[i], CV_8UC1);
        //normalize(channels[i], channels[i], min, max, NORM_MINMAX);
    //}
    //merge(channels,3,imgT);
    
}

void transLineal(Mat inputImg,Mat& imgT)
{
    //Mat ch1, ch2, ch3;
    //inputImg.convertTo(inputImg, CV_32FC1);
    Mat channels[3];
    Mat sum;
    Mat lab_image;
    double min, max;
    split(inputImg, channels);

    sum = channels[0] + channels[1] + channels[2];
    sum.convertTo(sum, CV_32FC1);
    //minMaxLoc(sum, &min, &max);
    //std::cout <<  max << std::endl;

    //std::cout << "sum : " << sum.size() << std::endl;
    for (int i = 0; i < 3; i++)
    {
        channels[i].convertTo(channels[i], CV_32FC1);
        //std::cout << "i :" << i <<" channels : " << channels[i].size() << std::endl;
        channels[i] /= sum;
        channels[i] *= 255;
        channels[i].convertTo(channels[i], CV_8UC1);
            //std::cout << "i :" << i <<" channels : " << channels[i].size() << std::endl;
        //minMaxLoc(channels[i], &min, &max);
        //std::cout <<  max << std::endl;
        //channels[i] = 255 * (channels[i] - min)/ (max - min);
        
        //normalize(channels[i], channels[i], min, max, NORM_MINMAX);
    }
    merge(channels,3,imgT);
    
}

void getMeanValue(Mat grayInp,float &meanPixels,float &stdPixels)
{
    //float meanPixels = 0.0;
    //float stdPixels = 0.0;
    meanPixels = 0.0;
    stdPixels = 0.0;
    float size = grayInp.rows * grayInp.cols;
    for (int i = 0; i < grayInp.rows; i++)
    {
        for(int j = 0; j < grayInp.cols; j++)
        {
            meanPixels += grayInp.at<uint8_t>(i,j)/size;
        }
    }
    for (int i = 0; i < grayInp.rows; i++)
    {
        for(int j = 0; j < grayInp.cols; j++)
        {
            stdPixels += ((grayInp.at<uint8_t>(i,j) - meanPixels) * (grayInp.at<uint8_t>(i,j) - meanPixels))/size;
        }
    }
    stdPixels = sqrt(stdPixels);
}