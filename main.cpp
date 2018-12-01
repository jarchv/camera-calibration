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
        Mat frame;
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
        imshow("foo", frame);
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
    imshow("threshold", threshold_output);

    findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE, Point(0, 0));

    /// Find the rotated rectangles and ellipses for each contour
    vector<RotatedRect> minRect( contours.size() );
    vector<RotatedRect> minEllipse( contours.size() );

    for( int i = 0; i < contours.size(); i++ )
    {
        minRect[i] = minAreaRect( Mat(contours[i]) );
        if( contours[i].size() > 5 )
        {
            minEllipse[i] = fitEllipse( Mat(contours[i]) );
        }
    }

  /// Draw contours + rotated rects + ellipses
    Mat drawing = Mat::zeros( threshold_output.size(), CV_8UC3 );
    for( int i = 0; i< contours.size(); i++ )
    {
        //Scalar color = Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
        Scalar color = Scalar( 255, 255, 0);
        // contour
        //drawContours( drawing, contours, i, color, 1, 8, vector<Vec4i>(), 0, Point() );
        // ellipse
        //ellipse( drawing, minEllipse[i], color, 2, 8 );
        // rotated rectangle
        Point2f rect_points[4]; 
        minRect[i].points( rect_points );
        
        int dsum[4];
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
        }
    }
    
  /// Show in a window
    namedWindow( "Contours", CV_WINDOW_AUTOSIZE );
    imshow( "Contours", drawing );

}
