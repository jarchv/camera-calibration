#include "utils.h"

double max_thresh  = 255;


cv::Mat frame;
cv::Mat bin;
cv::Mat gray;
cv::Mat contours;
cv::Mat result;
cv::Mat Template;

clock_t begin_time;

std::vector<cv::Point> RpdCnts;

int countFrame      = 0;
int T_width;
int T_height;
float temp_time;
float time_avr      = 0;




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

    
    Template    = cv::Mat(T_height*2 + 30, T_width*3 + 40, CV_8UC3, cv::Scalar(45,45,45));

    
    for(;;)
    {
        
        usleep(10000);
        cap >> frame;

        if(frame.empty())
            break;

        begin_time = clock();

        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray, gray, cv::Size(3,3), 0, 0);  

        result      = findCenters(frame, gray, bin, contours, countFrame, RpdCnts);

        temp_time   = float( clock () - begin_time ) /  CLOCKS_PER_SEC;
        time_avr    += temp_time;
        
    
        std::cout << "time per frame is " << time_avr/countFrame << std::endl;

        cv::resize(frame, frame, cv::Size(T_width, T_height));
        cv::cvtColor(gray, gray, cv::COLOR_GRAY2BGR);
        cv::resize(gray , gray , cv::Size(T_width, T_height));
        cv::cvtColor(bin, bin, cv::COLOR_GRAY2BGR);
        cv::resize(bin , bin , cv::Size(T_width, T_height));
        cv::cvtColor(contours, contours, cv::COLOR_GRAY2BGR);
        cv::resize(contours , contours , cv::Size(T_width, T_height));
        cv::resize(result , result , cv::Size(T_width, T_height));

        Mat2Mat(frame   , Template, 10              ,               10);
        Mat2Mat(gray    , Template, 10              ,     T_width + 20);
        Mat2Mat(bin     , Template, 10              ,   T_width*2 + 30);
        Mat2Mat(contours, Template, 20 + T_height   ,               10);
        Mat2Mat(result  , Template, 20 + T_height   ,     T_width + 20);


        cv::putText(Template,"Time per frame: " + std::to_string(float( clock () - begin_time ) /  CLOCKS_PER_SEC) + " seconds" , cv::Point(40, 30),cv::FONT_ITALIC,0,(0,0,255),3); 
        cv::imshow("Template", Template);
        char k = cv::waitKey(1);
        if ( k == 27) 
            break;
    }

    cap.release();

    return 0;
}
