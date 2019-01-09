#include "utils.h"
#include "base.h"

cv::Mat frame;
cv::Mat bin;
cv::Mat gray;

cv::Mat contours_draw;
cv::Mat result;
cv::Mat Template;

clock_t begin_time;
std::vector<cv::Point> RpdCnts;
std::vector<cv::Point3i> PointsTracking;
std::vector<std::vector<cv::Point>> contours;

std::vector<cv::Point2f> pointBuf;
std::vector<std::vector<cv::Point2f>> imagePoints;

cv::Mat cameraMatrix;
cv::Mat distCoeffs;

cv::Size imgSize;
int main(int argc, char** argv)
{
    std::string filename = argv[1];
    cv::VideoCapture cap("../files/"+filename);

    if (!cap.isOpened())
    {
        std::cout << "Failed to open camera." << std::endl;
        return -1;
    }

    cap >> frame;

    T_width  = (int)frame.cols*0.8;
    T_height = (int)frame.rows*0.8;
    
    //cv::VideoWriter video("../files/outcpp.avi",CV_FOURCC('D','I','V','3'),30, cv::Size( T_width*3 + 40,T_height*2 + 30)); 
    cv::Scalar color = cv::Scalar( 255, 250, 50);
    for(;;)
    {
        Template = cv::Mat(T_height*2 + 30, T_width*3 + 40, CV_8UC3, cv::Scalar(45,45,45));
        //usleep(10000);
        cap >> frame;
        imgSize = frame.size();
        if(frame.empty())
            break;

        begin_time = clock();

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray, gray, cv::Size(3,3), 0, 0);  

        result      = findCenters(frame, gray, bin, contours_draw, contours, countFrame, RpdCnts, PointsTracking, predict);

        temp_time    = float( clock () - begin_time ) /  CLOCKS_PER_SEC;
        time_avr    += temp_time;
        time_elapsed = time_avr/countFrame;

        erros += abs(predict - ground_truth);
        accuracy = 1 - erros/((float)countFrame * ground_truth);
        //std::cout << "time per frame is " << time_elapsed << "pred :" << accuracy <<std::endl;
        //std::cout << "osd " <<  PointsTracking.size() << std::endl;
        
        for (int i = 0; i < PointsTracking.size(); i++)
        {
            cv::circle(result, cv::Point(PointsTracking[i].y, PointsTracking[i].z), 2, cv::Scalar(0,0,255), 2, 8);  
            putText(result,std::to_string(PointsTracking[i].x),cv::Point(PointsTracking[i].y, PointsTracking[i].z),cv::FONT_ITALIC,0.8,color,2);             
        }
        
        cv::cvtColor(gray       , gray      , cv::COLOR_GRAY2BGR);
        cv::cvtColor(bin        , bin       , cv::COLOR_GRAY2BGR);
        //cv::cvtColor(contours   , contours  , cv::COLOR_GRAY2BGR);
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


        cv::imshow("Template", Template);

        //video.write(Template);
        char k = cv::waitKey(30);

        if ( k == 27) { break; }
        
        else if ( k == 'c' || k == 'C') 
        {
            std::cout << "Capturing image" << std::endl;
            if (PointsTracking.size() == 12)
            {
                for (int i = 0; i < PointsTracking.size(); i++)
                {
                    pointBuf.push_back(cv::Point2f(PointsTracking[i].y, PointsTracking[i].z));          
                }
                imagePoints.push_back(pointBuf);
            }
        }

        if ( mode == CAPTURING && imagePoints.size() >= 10 )
        {
            std::cout << "run calibrarion ..." << std::endl;
            bool result = SaveParams(imgSize,cameraMatrix,distCoeffs,imagePoints);
            mode = CALIBRATED;
            std::cout << "Result = " << result << std::endl;
        }

    }

    cap.release();
    //video.release();
    cv::destroyAllWindows();
    return 0;
}
