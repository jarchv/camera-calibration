#include <stdio.h>
#include <opencv2/opencv.hpp>
//#include <opencv2/imgproc.hpp>
//#include <opencv2/highgui.hpp>
#include "opencv2/imgcodecs.hpp"

using namespace cv;
using namespace std;

int main(int argc, char** argv)
{
    VideoCapture cap("calibration_kinectv2.avi");

    if (!cap.isOpened())
    {
        cout << "Failed to open camera." << endl;
        return -1;
    }
    namedWindow("foo",1);

    Mat edges;
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
        Point2f pc(frame.cols/2.,frame.rows/2.);
        
        // Matriz de rotacion
        rot = cv::getRotationMatrix2D(pc, -45, 1.0);

        warpAffine(frame, dst, rot, frame.size());
        cvtColor(dst, edges, COLOR_BGR2GRAY);
        GaussianBlur(edges, edges, Size(7,7), 1.5, 1.5);
        //Canny(edges, edges, 0, 30, 3);
        
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
            circle(dst, center, radius, Scalar(255,0,255), 3, LINE_AA);
        }

        resize(dst, dst2, cv::Size(dst.cols*0.5, dst.rows*0.5));
        char k = waitKey(1);
        imshow("foo", dst2);
        if (k == 27) 
            break;
    }

    cap.release();
}

