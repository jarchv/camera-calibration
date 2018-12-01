//#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
//#include "opencv2/imgcodecs.hpp"
#include <stdio.h>
#include <stdlib.h>

#include <iostream>
using namespace cv;
using namespace std;

int thresh = 100;
int max_thresh = 255;
Mat gray;
RNG rng(12345);

void thresh_callback(int, void* );
        Mat frame;
int main(int argc, char** argv)
{
    VideoCapture cap("files/calibration_kinectv2.avi");

    if (!cap.isOpened())
    {
        cout << "Failed to open camera." << endl;
        return -1;
    }
    namedWindow("foo",1);

    Mat rot;
    Mat dst;
    Mat dst2;
    //Mat dsize = Size(300, 300);

    for(;;)
    {

        cap >> frame;
        if(frame.empty())
            break;
        
        // Punto de referencia
        //Point2f pc(frame.cols/2.,frame.rows/2.);
        
        // Matriz de rotacion
        //rot = cv::getRotationMatrix2D(pc, -45, 1.0);

        //warpAffine(frame, dst, rot, frame.size());
        cvtColor(frame, gray, COLOR_BGR2GRAY);
        blur( gray, gray, Size(3,3) );
        
        //GaussianBlur(gray, gray, Size(7,7), 1.5, 1.5);
        //Canny(edges, edges, 0, 30, 3);
        /*
        vector<Vec3f> circles;

        HoughCircles(edges, circles, HOUGH_GRADIENT, 1, edges.rows/20, 200, 100, 0, 0 );
       
        
        for( size_t i = 0; i < circles.size(); i++ )
        {
            Vec3i c = circles[i];
            Point center = Point(c[0], c[1]);
            // circle center
            circle(frame, center, 1, Scalar(0,100,100), 3, LINE_AA);
            // circle outline
            int radius = c[2];
            circle(frame, center, radius, Scalar(255,0,255), 3, LINE_AA);
        }
        */
        //resize(dst, dst2, cv::Size(dst.cols*0.5, dst.rows*0.5));
        char k = waitKey(1);
        //imshow("foo", frame);
        createTrackbar( " Threshold:", "foo", &thresh, max_thresh, thresh_callback );
        thresh_callback( 0, 0 );
        if (k == 27) 
            break;
    }

    cap.release();
}

void thresh_callback(int, void* )
{
    Mat threshold_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;

    /// Detect edges using Threshold
    threshold( gray, threshold_output, thresh, 255, THRESH_BINARY );

    /// Find contours 
    //imshow("threshold", threshold_output);

    findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE, Point(0, 0));
    vector<Point> centers(contours.size());
    vector<float> radius(contours.size());
    /// Find the rotated rectangles and ellipses for each contour
    vector<RotatedRect> minRect( contours.size() );
    vector<RotatedRect> minEllipse( contours.size() );
    int counte = 0;
    for( int i = 0; i < contours.size(); i++ )
    {
        //minRect[i] = minAreaRect( Mat(contours[i]) );
        if( contours[i].size() > 26  && contours[i].size() < 150)
        {
            minRect[counte] = minAreaRect( Mat(contours[i]) );
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
  /// Draw contours + rotated rects + ellipses
    Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
    for( int i = 0; i< counte; i++ )
    {
        //Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        color = Scalar( 255, 255, 0);
        // contour
        //drawContours( drawing, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
        // ellipse
        //ellipse( drawing, minEllipse[i], color, 2, 8 );
        // rotated rectangle
        minRect[i].points( rect_points );
        centerx = 0.5*(rect_points[0].x + rect_points[2].x);
        centery = 0.5*(rect_points[0].y + rect_points[2].y);
        centers[i] = Point(centerx,centery); 
        radius[i] =  sqrt((rect_points[2].x - rect_points[0].x)*(rect_points[2].x - rect_points[0].x)+(rect_points[2].y - rect_points[0].y)*(rect_points[2].y - rect_points[0].y));
        /*int dsum[4];
        for (int j = 0; j < 4; j++)
        {
            int x2 = (rect_points[j].x - rect_points[(j + 2)%4].x) * (rect_points[j].x - rect_points[(j + 2)%4].x);
            int y2 = (rect_points[j].y - rect_points[(j + 2)%4].y) * (rect_points[j].y - rect_points[(j + 2)%4].y); 
            dsum[j]= sqrt(x2 + y2);
        }
        bool isEllipse = true;

        for (int k = 0; k < 4; k++)
        {
            if (dsum[k] > 90 || dsum[k] < 20)
                isEllipse = false;
        }

        if (isEllipse == true) {
            ellipse( drawing, minEllipse[i], color, 1.2, 8);
            //for( int j = 0; j < 4; j++ )
            //    line( drawing, rect_points[j], rect_points[(j+1)%4], color, 1, 8 );
        }*/
    }
        //std::cout << "tam " << contours.size() << std::endl;
    for(size_t p = 0;p<counte;p++){
        //std::cout << p << std::endl;
        //circle(drawing, centers[p], 10, Scalar(255,0,255), 3, LINE_AA);
        minRect[p].points( rect_points );
        centerx = 0.5*(rect_points[0].x + rect_points[2].x);
        centery = 0.5*(rect_points[0].y + rect_points[2].y);
        int count = 0;
        radiustemp = sqrt(pow((rect_points[2].x - rect_points[0].x),2)+pow((rect_points[2].y - rect_points[0].y),2));
        for(size_t k = 0;k<counte;k++)
        {
            if(k != p){
                float dist = sqrt((centerx - centers[k].x)*(centerx - centers[k].x) + (centery - centers[k].y)*(centery - centers[k].y));
                //std::cout << dist << std::endl;

                if((dist <= errormax) && (abs(radiustemp - radius[k])>errormaxradio))
                {
                    //ellipse( drawing, minEllipse[p], color, 1.2, 8);
                    count++;
                
                }
                if(count > 0){

                    ellipse( frame, minEllipse[p], color, 2, 8);
                    std::cout << p << " - " << counte << std::endl;
                    break;
                
                } 
            }
        }
    }
  /// Show in a window
    //namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
    //imshow( "Contours", drawing );
    imshow("foo", frame);
}
