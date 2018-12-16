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
std::vector<std::vector<cv::Point>> contours;

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
    
    for(;;)
    {
        Template = cv::Mat(T_height*2 + 30, T_width*3 + 40, CV_8UC3, cv::Scalar(45,45,45));
        //usleep(10000);
        cap >> frame;

        if(frame.empty())
            break;

        begin_time = clock();

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray, gray, cv::Size(3,3), 0, 0);  

        result      = findCenters(frame, gray, bin, contours_draw, contours, countFrame, RpdCnts,predict);

        temp_time    = float( clock () - begin_time ) /  CLOCKS_PER_SEC;
        time_avr    += temp_time;
        time_elapsed = time_avr/countFrame;

        erros += abs(predict - ground_truth);
        accuracy = 1 - erros/((float)countFrame * ground_truth);
        //std::cout << "time per frame is " << time_elapsed << "pred :" << accuracy <<std::endl;


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
        char k = cv::waitKey(1);
        if ( k == 27) 
            break;
    }

    cap.release();

    return 0;
}
