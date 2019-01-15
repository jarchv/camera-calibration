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

cv::Size imgSize;
cv::Size BoardSize(5,4);

double avr_error;

std::vector<cv::Point3f> PointsPositions;

int main(int argc, char** argv)
{
    std::string filename = argv[1];
    cv::VideoCapture cap("../files/"+filename);

    bool newF = true;

    if (!cap.isOpened())
    {
        std::cout << "Failed to open camera." << std::endl;
        return -1;
    }

    cap >> frame;
    
    T_width  = (int)frame.cols*0.8;
    T_height = (int)frame.rows*0.8;
    
    //cv::VideoWriter video("../files/outcpp.avi",CV_FOURCC('D','I','V','3'),30, cv::Size( T_width*3 + 40,T_height*2 + 30)); 
    
    for(;;)
    {
        Template = cv::Mat(T_height*2 + 30, T_width*3 + 40, CV_8UC3, cv::Scalar(45,45,45));
        
        cap >> frame;
        cv::Mat toModel = frame.clone();
        imgSize = frame.size();
        usleep(10000);
        if(frame.empty())
            break;

        begin_time = clock();

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray, gray, cv::Size(5,5), 0, 0);  

        result      = findCenters(frame, gray, bin, contours_draw, contours, countFrame, SortedPoints, BoardSize);

        temp_time    = float( clock () - begin_time ) /  CLOCKS_PER_SEC;
        time_avr    += temp_time;
        time_elapsed = time_avr/countFrame;

        if (SortedPoints.size() != (BoardSize.width * BoardSize.height))
            erros++;
    
        accuracy = 1 - erros/((float)countFrame * ground_truth);

        if (SortedPoints.size() == (BoardSize.width * BoardSize.height))
            drawLines(result, SortedPoints);

        cv::cvtColor(gray       , gray      , cv::COLOR_GRAY2BGR);
        cv::cvtColor(bin        , bin       , cv::COLOR_GRAY2BGR);
        
        for (int ic = 0; ic < contours.size(); ic++)
        {
            cv::Scalar color = cv::Scalar(55,55,255);
            cv::drawContours(contours_draw, contours, ic, color, 2, 8, -1, 0, cv::Point() );
        }

        cv::resize(frame        , frame         , cv::Size(T_width, T_height));
        cv::resize(gray         , gray          , cv::Size(T_width, T_height));
        cv::resize(bin          , bin           , cv::Size(T_width, T_height));  
        cv::resize(contours_draw, contours_draw , cv::Size(T_width, T_height));
        cv::resize(result       , result        , cv::Size(T_width, T_height));

        Mat2Mat(frame        , Template, 10              ,               10);
        Mat2Mat(gray         , Template, 10              ,     T_width + 20);
        Mat2Mat(bin          , Template, 10              ,   T_width*2 + 30);
        Mat2Mat(contours_draw, Template, 20 + T_height   ,               10);
        Mat2Mat(result       , Template, 20 + T_height   ,     T_width + 20);

                
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
                                + " seconds" , cv::Point(1100, 500),cv::  FONT_ITALIC,0.5,cv::Scalar(255,255,255),1); 
        cv::putText(Template,"Fps          : " 
                                + std::to_string(1/time_elapsed)
                                , cv::Point(1100, 520),cv::  FONT_ITALIC,0.5,cv::Scalar(255,255,255),1); 
        cv::putText(Template,"Accuracy     : " 
                                + std::to_string(accuracy)
                                , cv::Point(1100, 540),cv::  FONT_ITALIC,0.5,cv::Scalar(255,255,255),1);


        

        //video.write(Template);
        char k = cv::waitKey(30);

        if ( k == 27) { break; }
        
        else if ( k == 'c' || k == 'C') 
        {
            if (SortedPoints.size() == (BoardSize.width * BoardSize.height))
            {
                std::vector<cv::Point2f> pointBuf(20);
                for (int i = 0; i < SortedPoints.size(); i++)
                {
                    pointBuf[i] = cv::Point2f(SortedPoints[i].x, SortedPoints[i].y);          
                }
                imagePoints.push_back(pointBuf);
                std::cout << "Capturing image | current size = " << imagePoints.size() << std::endl;
            }
        }

        if ( mode == CAPTURING && imagePoints.size() >= 14 )
        {
            std::cout << "\nrun calibrarion ..." << std::endl;
            bool result = GetParams(imgSize,cameraMatrix,distCoeffs,imagePoints,avr_error,BoardSize,PointsPositions);
            
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
        }

        if ( mode == CALIBRATED)
        {
            /*
            cv::Mat view = frame.clone();
            cv::Mat temp = view.clone();


            cv::undistort(temp, view, cameraMatrix, distCoeffs);
            */
            cv::Mat view = toModel.clone();
            cv::Mat temp = view.clone();

            cv::undistort(temp, view, cameraMatrix, distCoeffs);

            
            cv::putText(Template,"fx     : " + std::to_string(cameraMatrix.at<double>(0,0)), 
                                  cv::Point(1100, 580),cv::  FONT_ITALIC,0.5,cv::Scalar(255,255,255),1);
            cv::putText(Template,"fy     : " + std::to_string(cameraMatrix.at<double>(1,1)), 
                                  cv::Point(1100, 600),cv::  FONT_ITALIC,0.5,cv::Scalar(255,255,255),1);
            cv::putText(Template,"cx     : " + std::to_string(cameraMatrix.at<double>(0,2)), 
                                  cv::Point(1100, 620),cv::  FONT_ITALIC,0.5,cv::Scalar(255,255,255),1);
            cv::putText(Template,"cy     : " + std::to_string(cameraMatrix.at<double>(1,2)), 
                                  cv::Point(1100, 640),cv::  FONT_ITALIC,0.5,cv::Scalar(255,255,255),1);
            cv::putText(Template,"Avr Re-Proj. Error    : " + std::to_string(avr_error), 
                                  cv::Point(1100, 680),cv::  FONT_ITALIC,0.5,cv::Scalar(255,255,255),1);
            
            if (SortedPoints.size() == (BoardSize.width * BoardSize.height))
            {
                std::vector<cv::Point3f> ObjectPointsModel(4);
                std::vector<cv::Point2f> imagePointsModel(4);
                std::vector<cv::Point2f> ObjectPointsProjected;
                std::vector<cv::Point>   ObjectPointsProjected2Image(4);

                ObjectPointsModel[0] = PointsPositions[0];
                ObjectPointsModel[1] = PointsPositions[6];
                ObjectPointsModel[2] = PointsPositions[1];
                ObjectPointsModel[3] = PointsPositions[5];

                imagePointsModel[0]  = cv::Point2f((float)SortedPoints[0].x,(float)SortedPoints[0].y);
                imagePointsModel[1]  = cv::Point2f((float)SortedPoints[6].x,(float)SortedPoints[6].y);
                imagePointsModel[2]  = cv::Point2f((float)SortedPoints[1].x,(float)SortedPoints[1].y);
                imagePointsModel[3]  = cv::Point2f((float)SortedPoints[5].x,(float)SortedPoints[5].y);                

                cv::Mat rvec(3,1,cv::DataType<double>::type);
                cv::Mat tvec(3,1,cv::DataType<double>::type);
                
                cv::solvePnP(ObjectPointsModel, imagePointsModel, cameraMatrix, distCoeffs, rvec, tvec);


                ObjectPointsModel.push_back(cv::Point3f(0.0,0.0,45.7));
                cv::projectPoints(ObjectPointsModel, rvec, tvec, cameraMatrix, distCoeffs, ObjectPointsProjected);

                bool neg = false;
                for(int i = 0; i < ObjectPointsProjected.size(); i++)
                {
                    //std::cout << ObjectPointsProjected[i] << std::endl;
                    if (ObjectPointsProjected[i].x < 0 || ObjectPointsProjected[i].y < 0 )
                    {
                        neg = true;
                        break;
                    }
                    ObjectPointsProjected2Image[i] = cv::Point((int)ObjectPointsProjected[i].x, (int)ObjectPointsProjected[i].y);
                    //std::cout << ObjectPointsProjected2Image[i] << std::endl;
                }

                if (neg == false)
                {
                    cv::line(view, ObjectPointsProjected2Image[0], ObjectPointsProjected2Image[3], cv::Scalar(255,0,0), 4, 8);
                    cv::line(view, ObjectPointsProjected2Image[0], ObjectPointsProjected2Image[2], cv::Scalar(0,0,255), 4, 8);
                    cv::line(view, ObjectPointsProjected2Image[0], ObjectPointsProjected2Image[4], cv::Scalar(0,255,0), 4, 8);
                }

            }
            cv::resize(view       , view        , cv::Size(T_width, T_height));
            Mat2Mat(view       , Template, 20 + T_height   ,     T_width*2 + 30);

        }

        cv::imshow("Template", Template);
    }

    cap.release();
    //video.release();
    cv::destroyAllWindows();
    return 0;
}
