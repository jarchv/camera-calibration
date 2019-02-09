#include "utils.h"
#include "base.h"

cv::Mat frame;
cv::Mat bin;
cv::Mat gray;
cv::Mat view;

cv::Mat contours_draw;
cv::Mat result;
cv::Mat Template;

clock_t begin_time;
std::vector<cv::Point> SortedPoints;
std::vector<cv::Point3i> PointsTracking;
std::vector<std::vector<cv::Point>> contours;


std::vector<std::vector<cv::Point2f>> imagePoints;

cv::Mat cameraMatrix;
cv::Mat distCoeffs;
cv::Mat temp;
cv::Size imgSize;

//float D = 45.7;
float D = 45;

double avr_error;
std::vector<cv::Point3f> PointsPositions;
std::vector<cv::Point3f> ObjectPointsModel;
std::vector<cv::Point2f> imagePointsModel;

std::vector<cv::Mat> imgToCalib;
int main(int argc, char** argv)
{
    
    if (argc != 4)
    {
        std::cout << "\nWe need 3 inputs:\n" << std::endl;
        std::cout << "\t - filename" << std::endl;
        std::cout << "\t - Pattern width" << std::endl;
        std::cout << "\t - Pattern height\n" << std::endl;
        return -1;
    }
    std::string filename = argv[1];
    int COLS = strtol (argv[2], NULL,10);
    int ROWS = strtol (argv[3], NULL,10);

    cv::VideoCapture cap("../files/"+filename);
    cv::Size BoardSize(COLS,ROWS);
    //cv::VideoCapture cap(0);

    bool newF = true;

    if (!cap.isOpened())
    {
        std::cout << "Failed to open camera." << std::endl;
        return -1;
    }

    cap >> frame;
    
    T_width  = (int)frame.cols*0.8;
    T_height = (int)frame.rows*0.8;
    std::cout << "size" << frame.size() << std::endl;
    cv::VideoWriter video("../files/outcpp.avi",CV_FOURCC('D','I','V','3'),30, cv::Size( T_width*3 + 40,T_height*2 + 30)); 
    
    for(;;)
    {
        Template = cv::Mat(T_height*2 + 30, T_width*3 + 40, CV_8UC3, cv::Scalar(45,45,45));
        
        cap >> frame;

        cv::Mat toModel = frame.clone();
        imgSize = frame.size();
        //usleep(10000);
        
        if(frame.empty())
            break;

        begin_time = clock();

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray, gray, cv::Size(5,5), 0, 0);  

        result      = findCenters(frame, gray, bin, contours_draw, contours, countFrame, SortedPoints, BoardSize, 0.08);
        
        temp_time    = float( clock () - begin_time ) /  CLOCKS_PER_SEC;
        time_avr    += temp_time;
        time_elapsed = time_avr/countFrame;

        if (SortedPoints.size() != (BoardSize.width * BoardSize.height))
            erros++;
        accuracy = 1 - erros/((float)countFrame);

        if (SortedPoints.size() == (BoardSize.width * BoardSize.height))
        {
            drawLines(result, SortedPoints);
        }
        cv::cvtColor(gray       , gray      , cv::COLOR_GRAY2BGR);
        cv::cvtColor(bin        , bin       , cv::COLOR_GRAY2BGR);
        
        
        for (int ic = 0; ic < contours.size(); ic++)
        {
            cv::Scalar color = cv::Scalar(55,55,255);
            cv::drawContours(contours_draw, contours, ic, color, 2, 8, -1, 0, cv::Point() );
        }

        cv::resize(frame        , frame         , cv::Size(T_width, T_height));
        //cv::resize(gray         , gray          , cv::Size(T_width, T_height));
        //cv::resize(bin          , bin           , cv::Size(T_width, T_height));  
        //cv::resize(contours_draw, contours_draw , cv::Size(T_width, T_height));
        cv::resize(result       , result        , cv::Size(T_width, T_height));

        Mat2Mat(frame        , Template, 10              ,               10);
        /*
        Mat2Mat(gray         , Template, 10              ,     T_width + 20);
        Mat2Mat(bin          , Template, 10              ,   T_width*2 + 30);
        Mat2Mat(contours_draw, Template, 20 + T_height   ,               10);
        Mat2Mat(result       , Template, 20 + T_height   ,     T_width + 20);
        */
        //Mat2Mat(bin         , Template, 10              ,     T_width + 20);
        Mat2Mat(result         , Template, 10              ,     T_width + 20);
        
        //Mat2Mat(result      , Template, 10              ,   T_width*2 + 30);
        //Mat2Mat(result , Template, 20 + T_height   ,               10);
        //Mat2Mat(result       , Template, 20 + T_height   ,     T_width + 20);

        //std::cout << "->" << std::endl;
        //std::string = "fps : " + std::to_string(1/time_avr);

        //cv::putText(Template,"Time per frame: " + std::to_string(float( clock () - begin_time ) /  CLOCKS_PER_SEC) + " seconds" , cv::Point(40, 30),cv::FONT_ITALIC,0,(0,0,255),3);
        /*
        if (countFrame == 30)
            std::cout << "time per frame is " << time_elapsed << " until " << countFrame<< " frames" << std::endl;
        else if (countFrame == 1e2)
            std::cout << "time per frame is " << time_elapsed << " until " << countFrame<< " frames" << std::endl;
        else if (countFrame == 2e2)
            std::cout << "time per frame is " << time_elapsed << " until " << countFrame<< " frames" << std::endl;
        else if (countFrame == 5e2)
            std::cout << "time per frame is " << time_elapsed << " until " << countFrame<< " frames" << std::endl;
        else if (countFrame == 1e3)
            std::cout << "time per frame is " << time_elapsed << " until " << countFrame<< " frames" << std::endl;
        else if (countFrame == 2e3)
            std::cout << "time per frame is " << time_elapsed << " until " << countFrame<< " frames" << std::endl;
        else if (countFrame == 5e3)
            std::cout << "time per frame is " << time_elapsed << " until " << countFrame<< " frames" << std::endl;

        */

        cv::putText(Template,"Time epalsed : " 
                                + std::to_string(time_elapsed)
                                + " seconds" , cv::Point(T_width + 40, T_height + 60),cv::  FONT_ITALIC,0.5,cv::Scalar(255,255,255),1); 
        cv::putText(Template,"Fps          : " 
                                + std::to_string(1/time_elapsed)
                                , cv::Point(T_width + 40, T_height +  80), cv::FONT_ITALIC, 0.5, cv::Scalar(255,255,255),1); 
        cv::putText(Template,"Accuracy     : " 
                                + std::to_string(accuracy)
                                , cv::Point(T_width + 40, T_height + 100), cv::FONT_ITALIC, 0.5, cv::Scalar(255,255,255),1);


        

        
        char k = cv::waitKey(30);

        if ( k == 27) { break; }
        
        else if ( k == 'c' || k == 'C') 
        {
            if (SortedPoints.size() == (BoardSize.width * BoardSize.height))
            {
                std::vector<cv::Point2f> pointBuf(BoardSize.width * BoardSize.height);
                for (int i = 0; i < SortedPoints.size(); i++)
                {
                    pointBuf[i] = cv::Point2f(SortedPoints[i].x, SortedPoints[i].y);          
                }
                imagePoints.push_back(pointBuf);
                std::cout << "Capturing image | current size = " << imagePoints.size() << std::endl;
                imgToCalib.push_back(toModel);
            }
        }

        if ( mode == CAPTURING && imagePoints.size() >= 12)
        {
            std::cout << "\nrun calibrarion ..." << std::endl;
            bool result = GetParams(imgToCalib,imgSize,cameraMatrix,distCoeffs,imagePoints,avr_error,BoardSize,PointsPositions);
            
            std::cout << "Result = " << result << std::endl;
            std::cout << "\nCalibration Matrix:  " << result << "\n"<<std::endl;
            
            for (int im = 0; im < 3; im++)
            {
                for (int jm = 0; jm < 3; jm++)
                {
                    std::cout << cameraMatrix.at<double>(im,jm) << " ";
                }
                std::cout << std::endl;
            }
            mode = CALIBRATED;
            std::cout << "\nDistCoeffs: " << distCoeffs << std::endl;
        }

        if ( mode == CALIBRATED)
        {
            /*
            cv::Mat view = frame.clone();
            cv::Mat temp = view.clone();


            cv::undistort(temp, view, cameraMatrix, distCoeffs);
            */
            cv::Mat view = toModel.clone();
            temp = view.clone();

            cv::undistort(temp, view, cameraMatrix, distCoeffs);

            
            cv::putText(Template,"fx     : " + std::to_string(cameraMatrix.at<double>(0,0)), 
                                  cv::Point(T_width+40, T_height + 140),cv::  FONT_ITALIC,0.5,cv::Scalar(255,255,255),1);
            cv::putText(Template,"fy     : " + std::to_string(cameraMatrix.at<double>(1,1)), 
                                  cv::Point(T_width+40, T_height + 160),cv::  FONT_ITALIC,0.5,cv::Scalar(255,255,255),1);
            cv::putText(Template,"cx     : " + std::to_string(cameraMatrix.at<double>(0,2)), 
                                  cv::Point(T_width+40, T_height + 180),cv::  FONT_ITALIC,0.5,cv::Scalar(255,255,255),1);
            cv::putText(Template,"cy     : " + std::to_string(cameraMatrix.at<double>(1,2)), 
                                  cv::Point(T_width+40, T_height + 200),cv::  FONT_ITALIC,0.5,cv::Scalar(255,255,255),1);
            cv::putText(Template,"Avr Re-Proj. Error    : " + std::to_string(avr_error), 
                                  cv::Point(T_width+40, T_height + 220),cv::  FONT_ITALIC,0.5,cv::Scalar(255,255,255),1);
            
            if (SortedPoints.size() == (BoardSize.width * BoardSize.height))
            {               
                ObjectPointsModel.clear();
                imagePointsModel.clear();

                int pos1 = 0;
                int pos2 = BoardSize.width * BoardSize.height - 1;
                int pos3 = BoardSize.width - 1;
                int pos4 = BoardSize.width * (BoardSize.height - 1);

                ObjectPointsModel.push_back(PointsPositions[pos1]);
                ObjectPointsModel.push_back(PointsPositions[pos2]);
                ObjectPointsModel.push_back(PointsPositions[pos3]);
                ObjectPointsModel.push_back(PointsPositions[pos4]);

                imagePointsModel.push_back(cv::Point2f((float)SortedPoints[pos1].x,(float)SortedPoints[pos1].y));
                imagePointsModel.push_back(cv::Point2f((float)SortedPoints[pos2].x,(float)SortedPoints[pos2].y));
                imagePointsModel.push_back(cv::Point2f((float)SortedPoints[pos3].x,(float)SortedPoints[pos3].y));
                imagePointsModel.push_back(cv::Point2f((float)SortedPoints[pos4].x,(float)SortedPoints[pos4].y)); 

                cv::Mat rvec(3,1,cv::DataType<double>::type);
                cv::Mat tvec(3,1,cv::DataType<double>::type);

                std::vector<cv::Point2f> ObjectPointsProjected;
                std::vector<cv::Point>   ObjectPointsProjected2Image;
                
                cv::solvePnP(ObjectPointsModel, imagePointsModel, cameraMatrix, distCoeffs, rvec, tvec);

                ObjectPointsModel.push_back(cv::Point3f(0.0,0.0,D));
                ObjectPointsModel.push_back(cv::Point3f(D,0.0,D));
                ObjectPointsModel.push_back(cv::Point3f(0.0,D,D));
                ObjectPointsModel.push_back(cv::Point3f(D,D,D));

                ObjectPointsModel.push_back(cv::Point3f(D  ,0.0,0));
                ObjectPointsModel.push_back(cv::Point3f(0.0,  D,0));
                ObjectPointsModel.push_back(cv::Point3f(  D,  D,0));

                cv::projectPoints(ObjectPointsModel, rvec, tvec, cameraMatrix, distCoeffs, ObjectPointsProjected);

                bool neg = false;
                for(int i = 0; i < ObjectPointsProjected.size(); i++)
                {
                    if (ObjectPointsProjected[i].x < 0 || ObjectPointsProjected[i].y < 0 )
                    {
                        neg = true;
                        break;
                    }
                    ObjectPointsProjected2Image.push_back(cv::Point((int)ObjectPointsProjected[i].x, (int)ObjectPointsProjected[i].y));
                }

                if (neg == false)
                {
                    cv::line(view, ObjectPointsProjected2Image[0], ObjectPointsProjected2Image[4], cv::Scalar(0,255,255), 4, 8);
                    cv::line(view, ObjectPointsProjected2Image[8], ObjectPointsProjected2Image[5], cv::Scalar(255,255,0), 4, 8);
                    cv::line(view, ObjectPointsProjected2Image[9], ObjectPointsProjected2Image[6], cv::Scalar(0,255,255), 4, 8);
                    cv::line(view, ObjectPointsProjected2Image[10], ObjectPointsProjected2Image[7], cv::Scalar(255,0,255), 4, 8);

                    cv::line(view, ObjectPointsProjected2Image[4], ObjectPointsProjected2Image[5], cv::Scalar(0,255,0), 4, 8);
                    cv::line(view, ObjectPointsProjected2Image[5], ObjectPointsProjected2Image[7], cv::Scalar(255,0,0), 4, 8);
                    cv::line(view, ObjectPointsProjected2Image[6], ObjectPointsProjected2Image[7], cv::Scalar(0,0,255), 4, 8);
                    cv::line(view, ObjectPointsProjected2Image[6], ObjectPointsProjected2Image[4], cv::Scalar(255,255,0), 4, 8);

                    cv::line(view, ObjectPointsProjected2Image[0 ], ObjectPointsProjected2Image[ 8], cv::Scalar(255,0,0), 4, 8);
                    cv::line(view, ObjectPointsProjected2Image[0 ], ObjectPointsProjected2Image[ 9], cv::Scalar(0,255,0), 4, 8);
                    cv::line(view, ObjectPointsProjected2Image[10], ObjectPointsProjected2Image[ 8], cv::Scalar(0,255,0), 4, 8);
                    cv::line(view, ObjectPointsProjected2Image[10], ObjectPointsProjected2Image[ 9], cv::Scalar(255,0,0), 4, 8);
                }
                
                std::vector<cv::Point2f> ObjectPointsModel2D;
                for(int i = 0; i < PointsPositions.size(); i++)
                {
                    ObjectPointsModel2D.push_back(cv::Point2f(PointsPositions[i].x + 200, PointsPositions[i].y + 100));
                }

                std::vector<cv::Point2f> imagePointsModel2D;

                for(int i = 0; i < SortedPoints.size(); i++)
                {
                    imagePointsModel2D.push_back(cv::Point2f(SortedPoints[i].x, SortedPoints[i].y));
                }

                cv::Mat M = cv::findHomography(imagePointsModel2D, ObjectPointsModel2D);
                cv::Mat dst;

                if (M.cols == 3 && M.rows == 3)
                {
                    warpPerspective(temp, dst, M, temp.size());

                    cv::flip(dst,dst,0);
                    cv::resize(dst, dst, cv::Size(T_width, T_height));
                    Mat2Mat(dst      , Template, 10              ,   T_width*2 + 30);

                    std::vector<cv::Point> SortedPoints2;
                    cv::Mat gray2;
                    cv::Mat bin2;
                    cv::Mat contours_draw2;
                    std::vector<std::vector<cv::Point>> contours2;
                    int c = 0;

                    cv::cvtColor(dst, gray2, cv::COLOR_BGR2GRAY);
                    cv::GaussianBlur(gray2, gray2, cv::Size(5,5), 0, 0);  

                    cv::Mat result2 = findCenters(dst, gray2, bin2, contours_draw2, contours2, c, SortedPoints2, BoardSize, 0.05);

                    if (SortedPoints2.size() == (BoardSize.width * BoardSize.height))
                    {
                        drawLines(result2, SortedPoints2);

                        int Xl = (int)SortedPoints2[0].x;
                        int Yl = (int)SortedPoints2[15].y;

                        cv::Rect myROI(Xl - D * 0.5, Yl - D*0.5 , D * (BoardSize.width - 0.5), D * (BoardSize.height - 0.5));
                        cv::Mat croppedRef(result2, myROI);
                        cv::resize(croppedRef, croppedRef , cv::Size(T_width, T_height));

                        Mat2Mat(croppedRef , Template, 20 + T_height   ,               10);
                    }
                }

            }
            cv::resize(view       , view        , cv::Size(T_width, T_height));
            Mat2Mat(view       , Template, 20 + T_height   ,     T_width*2 + 30);            

        }

        cv::imshow("Template", Template);
        //video.write(Template);
    }

    cap.release();
    video.release();
    cv::destroyAllWindows();
    return 0;
}
