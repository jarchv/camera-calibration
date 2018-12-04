#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <iostream>
using namespace cv;
using namespace std;

int thresh      = 100;
int max_thresh  = 255;

Mat frame;
Mat gray;
Mat normImg;


Mat channels[3];
Mat lab_image;

int corners_prev[4] = {0, 0, 0, 0};

void findCenters(int, int);
void CLAHEtrans(Mat, Mat&);
void chrCoor(Mat, Mat&);
void getMeanPixel(Mat,float&,float &);
bool isIncluded(vector<Point>, Point );

int  main(int argc, char** argv)
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
        
        CLAHEtrans(frameLab2, imgT2);
        //imshow("clahe",imgT2);
        
        chrCoor(imgT2,imgT);
        //imshow("lineal",imgT);
        cvtColor(imgT, gray, COLOR_BGR2GRAY);
        
        // v1 : -30
        // v2 : +10
        
        getMeanPixel(gray,mean,std);
        
        double thresh_val = 105;
        //std::cout << "thresh_val : " << thresh_val << std::endl; 
        //blur( gray, gray, Size(3,3) );
        GaussianBlur(gray, gray, Size(3,3), 0, 0);

        char k = waitKey(1);
        findCenters((int)thresh_val, max_thresh);
        if (k == 27) 
            break;
    }

    cap.release();
}

bool isIncluded(vector<Point> X, Point Pt)
{
    for(size_t ii = 0; ii < X.size(); ii ++)
    {
        double dc = sqrt((X[ii].x - Pt.x)*(X[ii].x - Pt.x) + (X[ii].y - Pt.y)*(X[ii].y - Pt.y));
        if (dc <= 3)
            return true;
    }
    return false;
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
    vector<Point> pdCnts;
    vector<RotatedRect> minRect( contours.size() );
    vector<RotatedRect> minEllipse( contours.size() );
    int counte = 0;
    for( int i = 0; i < contours.size(); i++ )
    {
        if( contours[i].size() > 7  && contours[i].size() < 180)
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
    float radiustemp;
    float errormaxDiam = 7.0;
    float errormax = 3.0; 

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
                if((dist <= errormax) && (abs(radiustemp - radius[k])>errormaxDiam))
                {
                    count++;
                }
                if(count > 0){
                    ellipse( frame, minEllipse[p], color, 2, 8);
                    if (isIncluded(pdCnts, Point(centerx, centery)) == false)
                    {
                        pdCnts.push_back(Point(centerx, centery));
                        //ellipse( frame, minEllipse[p], color, 2, 8);
                    } 
                    break;
                } 
            }
        }
    }

    int corners[4] = {10000, 10000, 0, 0};
    vector<Point> cornerPoints(4);

    for(size_t pdi = 0; pdi < pdCnts.size(); pdi ++)
    {
        if (pdCnts[pdi].x < corners[0]){
            cornerPoints[0] = pdCnts[pdi];
            corners[0]      = pdCnts[pdi].x;
        }
        if (pdCnts[pdi].x > corners[2]){
            cornerPoints[2] = pdCnts[pdi];
            corners[2]      = pdCnts[pdi].x;
        }
        if (pdCnts[pdi].y < corners[1]){
            cornerPoints[1] = pdCnts[pdi];
            corners[1]      = pdCnts[pdi].y;
        }

        if (pdCnts[pdi].y > corners[3]){
            cornerPoints[3] = pdCnts[pdi]; 
            corners[3]      = pdCnts[pdi].y; 
        }    
        circle( frame, pdCnts[pdi], 2, cvScalar(0,0,255), 2, 8);  
        /*int nCtrs = 0;
        
        for(size_t pdj = 0; pdj < pdCnts.size(); pdj ++)
        {
            auto currC = pdCnts[pdj];
            auto distC = sqrt((pdCnts[pdi].x - currC.x)*(pdCnts[pdi].x  - currC.x) + (pdCnts[pdi].y - currC.y)*(pdCnts[pdi].y - currC.y));
            if (distC < 75){
                //std::cout << " ++" << std::endl;
                nCtrs++;
            }
        }
        std::cout << " count = " << nCtrs << std::endl;
        if (nCtrs == 3)
        {
            std::cout << " == 3" << std::endl;
            circle( frame, pdCnts[pdi], 2, cvScalar(0,0,255), 2, 8);  
        }
        //std::cout<< "X: " << pdCnts[pdi].x << "Y: " << pdCnts[pdi].y << std::endl;  
        */
    }
    for (int ic = 0; ic < 4; ic++)
    {
        corners[ic] = 0.9 *corners[ic] + 0.1 * corners_prev[ic];
        corners_prev[ic] = corners[ic];
    }
    
    //rectangle( frame, Point(corners[0],corners[1]),Point(corners[2],corners[3]), cvScalar(0,255,0), 2, 8);  
    imshow("foo", frame);
}

void CLAHEtrans(Mat inputImg,Mat& imgT)
{
    std::vector<cv::Mat> channels(3);
    split(inputImg, channels);

    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(3);
    cv::Mat dst;
    clahe->apply(channels[0], dst);
    dst.copyTo(channels[0]);
    cv::merge(channels, lab_image);

    cv::cvtColor(lab_image, imgT, CV_Lab2BGR);    
}

void chrCoor(Mat inputImg,Mat& imgT)
{
    double min, max;
    split(inputImg, channels);

    Mat sum = channels[0] + channels[1] + channels[2];
    sum.convertTo(sum, CV_32FC1);
    for (int i = 0; i < 3; i++) 
    {
        channels[i].convertTo(channels[i], CV_32FC1);
        channels[i] /= sum;
        channels[i] *= 255;
        channels[i].convertTo(channels[i], CV_8UC1);
        //minMaxLoc(channels[i], &min, &max);
        //std::cout << "min :" << min <<", max : " << max << std::endl;
        channels[i].convertTo(channels[i], CV_8UC1);
    }
    merge(channels,3,imgT);
    
}

void getMeanPixel(Mat grayInp,float &meanPixels,float &stdPixels)
{
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