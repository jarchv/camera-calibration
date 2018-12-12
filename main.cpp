#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include <stdlib.h>
//#include <time.h>
#include <unistd.h>
#include <iostream>
#include <omp.h>

using namespace cv;
using namespace std;


int thresh      = 100;
double max_thresh  = 255;

Mat frame;
Mat frameResult;
Mat gray;
Mat normImg;
Mat bin;    

float minc = 1000.0;
int cb = 0;
Mat channels[3];
Mat lab_image;

int corners_prev[4] = {0, 0, 0, 0};

void   findCenters(Mat,int, int);
void transCLAHE(Mat, Mat&);
void  transLineal(Mat, Mat&);
void getMeanValue(Mat,float&,float &);
bool isIncluded(vector<Point>, Point );
void thresholdIntegral(cv::Mat &inputMat, cv::Mat &outputMat);
clock_t begin_time;

Point minMBB = Point(115,80);
Point maxMBB = Point(434,340);
int countFrame = 0;

Mat prev_frame;
void getSquareImage(cv::InputArray img, cv::OutputArray dst, int size)
{
    if (size < 2) size = 2;
    int width = img.cols(), height = img.rows();

    cv::Mat square = dst.getMat();

    if (width == height) {
        cv::resize(img, square, Size(size, size));
        return;
    }

    square.setTo(Scalar::all(0));

    int max_dim = (width >= height) ? width : height;
    float scale = ((float)size) / max_dim;
    
    cv::Rect roi;

    if (width >= height)
    {
        roi.width = size;
        roi.x = 0;
        roi.height = (int)(height * scale);
        roi.y = (size - roi.height) / 2;
    }
    else
    {
        roi.y = 0;
        roi.height = size;
        roi.width = (int)(width * scale);
        roi.x = (size - roi.width) / 2;
    }

    cv::resize(img, square(roi), roi.size());
}
void showImages(const String& window_name, int rows, int cols, int size, std::initializer_list<Mat*> images, int pad = 1)
{
    if (pad <= 0) pad = 0;

    int width = size * cols + ((cols + 1) * pad);
    int height = size * rows + ((rows + 1) * pad);

    Mat dst = Mat(height, width, CV_8UC3, Scalar::all(255));

    int x = 0, y = 0, cols_counter = 0, img_counter = 0;

    for (auto& img : images) {
        Mat roi = dst(Rect(x + pad, y + pad, size, size));

        getSquareImage(*img, roi, size);

        x += roi.cols + pad;

        if (++cols_counter == cols) {
            cols_counter = x = 0;
            y += roi.rows + pad;
        }

        if (++img_counter >= rows * cols) break;
    }
    moveWindow(window_name,40,30);
    imshow(window_name, dst);
}
int main(int argc, char** argv)
{
    
    // do something
    
    string filename = argv[1];
    
    VideoCapture cap("files/"+filename);

    if (!cap.isOpened())
    {
        cout << "Failed to open camera." << endl;
        return -1;
    }
    //namedWindow("foo",1);

    Mat rot;
    Mat dst;
    Mat dst2;
    Mat imgT;
    Mat imgT2;
    Mat frameLab2;
    float mean;
    float std;
    Mat lab;

    cap >> prev_frame;

    Mat pgray;
    Mat pgray_f;
    Mat gray_f;
    cvtColor(prev_frame, pgray, COLOR_BGR2GRAY);
    //pgray.convertTo(pgray_f, CV_32FC1);


    Mat diff;
    for(;;)
    {
        begin_time = clock();
        //usleep(10000);
        cap >> frame;
        cap >> frameResult;
        if(frame.empty())
            break;
        //cv::cvtColor(frame, frame, CV_BGR2Lab);
        //transCLAHE(frame,frame);

        cvtColor(frame, gray, COLOR_BGR2GRAY);

        //cv::absdiff(frame, prev_frame, diff);

            //normalize(diff,diff , 0, 255);
        //diff.convertTo(diff, CV_8UC1);
        //cvtColor(diffDis, diff, COLOR_BGR2GRAY);
        
        //std::cout << "counFrame" << countFrame << std::endl;
        //cap >> prev_frame;
        
        /*Mat foregroundMask = cv::Mat::zeros(diff.rows, diff.cols, CV_8UC3);

        float threshold =30.0f;
        float dist;
        //cvtColor(frame, pgray, COLOR_BGR2GRAY);
        for(int j=0; j<diff.rows; ++j)
            for(int i=0; i<diff.cols; ++i)
            {
                cv::Vec3b pix = diff.at<cv::Vec3b>(j,i);

                dist = (pix[0]*pix[0] + pix[1]*pix[1] + pix[2]*pix[2]);
                dist = sqrt(dist);

                if(dist>threshold)
                {
                    foregroundMask.at<unsigned char>(j,i) = 255;
                }
            }*/
        //Mat foregroundMask = cv::Mat::zeros(diff.rows, diff.cols, CV_8UC1);
        //gray = diff + foregroundMask;
        //cvtColor(foregroundMask, foregroundMask, CV_8UC1);
        //Canny(foregroundMask, foregroundMask, 50, 200);

   
        //foregroundMask.convertTo(gray, CV_32FC3, 1/255.0);
        //cvtColor(gray, gray, COLOR_BGR2GRAY);
        //     imshow("diff",gray);
        //if CLAHE
        /*
        cv::cvtColor(frame, frameLab2, CV_BGR2Lab);
        
        transCLAHE(frameLab2, imgT2);
        //imshow("clahe",imgT2);
        

        //imshow("lineal",imgT);

        
        // v1 : -30
        // v2 : +10
        
        getMeanValue(gray,mean,std);
        
        double thresh_val = 105;
        //std::cout << "thresh_val : " << thresh_val << std::endl; 
        
        //blur( gray, gray, Size(3,3) );*/

        GaussianBlur(gray, gray, Size(3,3), 0, 0);  
        double thresh_val = 105;
        char k = waitKey(1);
        findCenters(frameResult,(int)thresh_val, max_thresh);
        if (k == 27) 
            break;
    }

    cap.release();
}

bool isIncluded(vector<Point> X, Point Pt)
{
    for(size_t ii = 0; ii < X.size(); ii ++)
    {
        //std::cout << "in" <<std::endl;
        double dc = sqrt((X[ii].x - Pt.x)*(X[ii].x - Pt.x) + (X[ii].y - Pt.y)*(X[ii].y - Pt.y));
        if (dc <= 4)
            return true;
    }
    return false;
}


vector<Point> RpdCnts;
void findCenters(Mat frameResult,int thresh, int max_thresh)
{
    Mat threshold_output;
    vector<vector<Point> > contours;
    vector<Vec4i> hierarchy;
    vector<Point> pdCnts(13);
    //threshold( gray, threshold_output, thresh, max_thresh, THRESH_BINARY );
    //iplimage_from_cvmat(&frame,iplImageFrame);
    //iplimage_from_cvmat(&gray,iplImageGray);
    //iplImageFrame = new IplImage(frame);
    //iplImage = new IplImage();
    //iplImageGray = cvCreateImage(cvSize(frame.size().width, frame.size().height), 8, 1);
    //    std::cout << " sdds " << std::endl;
    //iplImageGray = cvCreateImage(CvSize(frame.size().width,frame.size().height),CV_8UC1,1);
    //cv::Mat grayscale(cv::Size(frame.size().width,frame.size().height),CV_8UC3);
    bin = Mat::zeros(gray.size(), CV_8UC1);
    thresholdIntegral(gray,bin);
    //bin = bing;
    //cv::adaptiveThreshold(frame,gray,max_thresh,);
    //cvShowImage("dd",iplImageGray);
    //gray = cvarrToMat(iplImageGray);
    moveWindow("threshold",640,640);
    imshow("threshold", bin);
    //Canny(gray, gray, 0, 200);
    Mat element = getStructuringElement( MORPH_ELLIPSE,
                                       Size( 7,7),
                                       Point( 0, 0 ) );
    dilate( bin, threshold_output, element );


    findContours( threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE, Point(0, 0));
    vector<Point> centers(contours.size());
    vector<float> radius(contours.size());

    vector<RotatedRect> minRect( contours.size() );
    vector<RotatedRect> minEllipse( contours.size() );
    int counte = 0;
    
    for( int i = 0; i < contours.size(); i++ )
    {
        if( contours[i].size() > 5  && contours[i].size() < 250)
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
    float errormaxDiam = 5.5;
    float errormax = 2; 

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
    int countc=0;
  
    for(int p = 0;p<counte;p++){
        minRect[p].points( rect_points );
        centerx = 0.5*(rect_points[0].x + rect_points[2].x);
        centery = 0.5*(rect_points[0].y + rect_points[2].y);
        int count = 0;
        radiustemp = sqrt(pow((rect_points[2].x - rect_points[0].x),2)+pow((rect_points[2].y - rect_points[0].y),2));
        for(int k = 0;k<counte;k++)
        {
            if(k != p){
                float dist = sqrt((centerx - centers[k].x)*(centerx - centers[k].x) + (centery - centers[k].y)*(centery - centers[k].y));
                if((dist <= errormax) && (radius[k] - radiustemp)  > errormaxDiam)
                {
                    count++;
                }
                if(count > 0){
                    //ellipse( frame, minEllipse[p], color, 2, 8);

                    if (isIncluded(pdCnts, Point(centerx, centery)) == false)
                    {
                        
                        if(countFrame>0){
                            float minc = 10000.0;
                            for(int c =  0;c<RpdCnts.size();c++){
                                float dist_centers = sqrt((centerx - RpdCnts[c].x)*(centerx - RpdCnts[c].x) + (centery - RpdCnts[c].y)*(centery - RpdCnts[c].y));
                                if(dist_centers<minc){
                                    minc = dist_centers;
                                    cb = c;
                                }
                            }
                            
                            //std::cout << "current " << countc << " predic " << cb << std::endl;
                            if(minc < 9)
                            {
                                circle( frameResult, Point(centerx, centery),2, cvScalar(0,0,255), 2, 8); 
                                putText(frameResult,to_string(cb),Point(centerx, centery),FONT_ITALIC,1.0,color,3); 
                                //pdCnts.push_back(Point(centerx, centery));
                                pdCnts[cb] = Point(centerx, centery);
                            }

                            else 
                            {
                                pdCnts[countc] = Point(centerx, centery);
                                //pdCnts.push_back(Point(centerx, centery));
                                circle( frameResult, Point(centerx, centery), 2, cvScalar(0,0,255), 2, 8);  
                                putText(frameResult,to_string(countc),Point(centerx, centery),FONT_ITALIC,1.0,color,3); 
                            }
                            countc++;
                        }
                        else{
                            pdCnts[countc] = Point(centerx, centery);
                            //pdCnts.push_back(Point(centerx, centery));
                            circle( frameResult, Point(centerx, centery), 2, cvScalar(0,0,255), 2, 8);  
                            countc++;
                        }
                        
                        //ellipse( frame, minEllipse[p], color, 2, 8);
                        
                    } 
                    break;
                } 
            }
        }
        //pdCnts.push_back(Point(0,0));
        //pdCnts.push_back(Point(0,0));
        //pdCnts.push_back(Point(0,0));
    }
    //std::cout << " size " << pdCnts << std::endl;
    RpdCnts = pdCnts;

 

    countFrame++;
    /*
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

    }
    for (int ic = 0; ic < 4; ic++)
    {
        corners[ic] = 0.9 *corners[ic] + 0.1 * corners_prev[ic];
        corners_prev[ic] = corners[ic];
    }
    
    //std::cout << corners[0] << ", "<< corners[1] << ", "<< corners[2] <<", "<< corners[3] << std::endl;
    
    //rectangle( frame, Point(corners[0],corners[1]),Point(corners[2],corners[3]), cvScalar(0,255,0), 2, 8);  
    
    minMBB = Point(corners[0],corners[1]);
    maxMBB = Point(corners[2],corners[3]);
    /*
    for (int ic = 0; ic < cornerPoints.size(); ic++)
    {
        RE( frame, cornerPoints[ic], cornerPoints[(ic+1)%4], cvScalar(0,255,0), 2, 8);  
    }*/
    std::cout << "time per frame is " <<float( clock () - begin_time ) /  CLOCKS_PER_SEC << std::endl;

    showImages("Results", 1, 2, 500, {&frame,&frameResult}, 3);
    putText(frameResult,"Time per frame: " + to_string(float( clock () - begin_time ) /  CLOCKS_PER_SEC) + " seconds" ,Point(40, 30),FONT_ITALIC,0,(0,0,255),3); 
    //imshow("foo", frame);

}

void transCLAHE(Mat inputImg,Mat& imgT)
{
    //inputImg.convertTo(inputImg, CV_32FC1);
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

void transLineal(Mat inputImg,Mat& imgT)
{
    double min, max;
    split(inputImg, channels);

    Mat sum = channels[0] + channels[1] + channels[2];
    sum.convertTo(sum, CV_32FC1);
    for (int i = 0; i < 3; i++) 
    {
        channels[i].convertTo(channels[i], CV_32FC1);
        //std::cout << "i :" << i <<" channels : " << channels[i].size() << std::endl;
        channels[i] /= sum;
        channels[i] *= 255;
        channels[i].convertTo(channels[i], CV_8UC1);
        minMaxLoc(channels[i], &min, &max);
        //std::cout << "min :" << min <<", max : " << max << std::endl;
        channels[i].convertTo(channels[i], CV_8UC1);
    }
    merge(channels,3,imgT);
    
}

void getMeanValue(Mat grayInp,float &meanPixels,float &stdPixels)
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
void thresholdIntegral(cv::Mat &inputMat, cv::Mat &outputMat)
{
    // accept only char type matrices
    CV_Assert(!inputMat.empty());
    CV_Assert(inputMat.depth() == CV_8U);
    CV_Assert(inputMat.channels() == 1);
    CV_Assert(!outputMat.empty());
    CV_Assert(outputMat.depth() == CV_8U);
    CV_Assert(outputMat.channels() == 1);

    // rows -> height -> y
    int nRows = inputMat.rows;
    // cols -> width -> x
    int nCols = inputMat.cols;

    // create the integral image
    cv::Mat sumMat;
    cv::integral(inputMat, sumMat);

    CV_Assert(sumMat.depth() == CV_32S);
    CV_Assert(sizeof(int) == 4);

    int S = MAX(nRows, nCols)/8;
    double T = 0.15;

    // perform thresholding
    int s2 = S/2;
    int x1, y1, x2, y2, count, sum;

    // CV_Assert(sizeof(int) == 4);
    int *p_y1, *p_y2;
    uchar *p_inputMat, *p_outputMat;
#pragma parallel for
    for( int i = 0; i < nRows; ++i)
    {
        y1 = i-s2;
        y2 = i+s2;

        if (y1 < 0){
            y1 = 0;
        }
        if (y2 >= nRows) {
            y2 = nRows-1;
        }

        p_y1 = sumMat.ptr<int>(y1);
        p_y2 = sumMat.ptr<int>(y2);
        p_inputMat = inputMat.ptr<uchar>(i);
        p_outputMat = outputMat.ptr<uchar>(i);

        for ( int j = 0; j < nCols; ++j)
        {
            // set the SxS region
            x1 = j-s2;
            x2 = j+s2;

            if (x1 < 0) {
                x1 = 0;
            }
            if (x2 >= nCols) {
                x2 = nCols-1;
            }

            count = (x2-x1)*(y2-y1);

            // I(x,y)=s(x2,y2)-s(x1,y2)-s(x2,y1)+s(x1,x1)
            sum = p_y2[x2] - p_y1[x2] - p_y2[x1] + p_y1[x1];

            if ((int)(p_inputMat[j] * count) < (int)(sum*(1.0-T)))
                p_outputMat[j] = 255;
            else
                p_outputMat[j] = 0;
        }
    }
}
