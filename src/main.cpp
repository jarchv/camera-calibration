#include "utils.h"
#include "base.h"
#include <fstream>

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

std::vector<double> ThetaArray;
std::vector<double> PhiXArray;
std::vector<double> PhiYArray;

//float D = 45.7;
float D = 45;

double avr_error;
std::vector<cv::Point3f> PatternPointsPositions;
std::vector<cv::Point3f> ObjectPointsModel;
std::vector<cv::Point2f> imagePointsModel;

std::vector<cv::Mat> imgToCalib;

std::string PATH = "../rings/";

cv::Mat rMat(3,3,cv::DataType<double>::type);

int main(int argc, char** argv)
{
    const int dir_err1 = system("mkdir -p ../rings");
    const int dir_err2 = system("mkdir -p ../rings/visualize");
    const int dir_err3 = system("mkdir -p ../rings/txt");
    const int dir_err4 = system("mkdir -p ../rings/modals");

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

    //cv::VideoCapture cap("../files/"+filename);
    cv::Size BoardSize(COLS,ROWS);
    cv::VideoCapture cap(0);

    bool newF = true;

    if (!cap.isOpened())
    {
        std::cout << "Failed to open camera." << std::endl;
        return -1;
    }

    cap >> frame;
    
    calcPointPosition(PatternPointsPositions, BoardSize);

    T_width  = (int)frame.cols*0.8;
    T_height = (int)frame.rows*0.8;
    std::cout << "size" << frame.size() << std::endl;
    cv::VideoWriter video("../files/outcpp.avi",CV_FOURCC('D','I','V','3'),30, cv::Size( T_width*3 + 40,T_height*2 + 30)); 

    int i = 0;

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
        cv::resize(result       , result        , cv::Size(T_width, T_height));

        Mat2Mat(frame        , Template, 10              ,               10);

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

        
        
        if (mode == CAPTURING)
            selfCapture(result,SortedPoints, toModel, imgToCalib, imagePoints, PatternPointsPositions, BoardSize,ThetaArray,PhiXArray,PhiYArray,false);
        
        if (mode != CALIBRATED)
        {
            cv::ellipse(result, cv::Point((int)(result.cols-40),
                                (int)(result.rows-40)), 
                                cv::Size(15,15), 0.0, 0.0, 
                                ThetaArray.size()*360.0/IMAGES_TO_CALIBRATE, 
                                cv::Scalar(10,10,250), 2, 8);
        }

        Mat2Mat(result         , Template, 10              ,     T_width + 20);
        if ( k == 27) { break; }

        
        else if ( k == 'c' || k == 'C')
        {
            if (mode == CAPTURING)
                selfCapture(result,SortedPoints, toModel, imgToCalib, imagePoints, PatternPointsPositions, BoardSize,ThetaArray,PhiXArray,PhiYArray,true);
        }

        else if ( k == 'r' || k == 'R')
        {
            mode = RECONSTRUCTION;
        }

        if ( mode == CAPTURING && imagePoints.size() >= IMAGES_TO_CALIBRATE)
        {
            std::cout << "\nrun calibrarion ..." << std::endl;
            bool result = GetParams(imgToCalib,imgSize,cameraMatrix,distCoeffs,imagePoints,avr_error,BoardSize,PatternPointsPositions);
            
            //std::cout << "Result = " << result << std::endl;
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
            //std::cout << "\nDistCoeffs: " << distCoeffs << std::endl;
            cap.set(CV_CAP_PROP_POS_FRAMES, 0);
        }

        if ( mode == CALIBRATED)
        {
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

                ObjectPointsModel.push_back(PatternPointsPositions[pos1]);
                ObjectPointsModel.push_back(PatternPointsPositions[pos2]);
                ObjectPointsModel.push_back(PatternPointsPositions[pos3]);
                ObjectPointsModel.push_back(PatternPointsPositions[pos4]);

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
                    CreateObject3D(view, ObjectPointsProjected2Image);                   
                }

                /*
                ***********************
                *  FRONT TO PARALLEL  *
                *  =================  *
                ***********************
                */
                std::vector<cv::Point2f> ObjectPointsModel2D;
                for(int i = 0; i < PatternPointsPositions.size(); i++)
                {
                    ObjectPointsModel2D.push_back(cv::Point2f(PatternPointsPositions[i].x + 200, PatternPointsPositions[i].y + 100));
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

        if (mode == RECONSTRUCTION)
        {
            if (SortedPoints.size() == (BoardSize.width * BoardSize.height))
            {
                //drawLines(result, SortedPoints);
                cv::Mat rvec(3,1,cv::DataType<double>::type);
                cv::Mat tvec(3,1,cv::DataType<double>::type);
                cv::Mat rt(3,4,cv::DataType<double>::type);
                
                imagePointsModel.clear();
                for (int i = 0; i < SortedPoints.size(); i++)
                {
                    imagePointsModel.push_back(cv::Point2f(SortedPoints[i].x, SortedPoints[i].y));
                }
                cv::solvePnP(PatternPointsPositions, imagePointsModel, cameraMatrix, distCoeffs, rvec, tvec);
                cv::Rodrigues(rvec, rMat);

                for(int jj = 0; jj < rt.cols; jj++)
                {
                    for(int ii = 0; ii < rt.rows; ii++)
                    {
                        if (jj == rt.cols -1)
                        {
                            rt.at<double>(ii,jj) = tvec.at<double>(ii,0);
                        }
                        else
                        {
                            rt.at<double>(ii,jj) = rMat.at<double>(ii,jj);
                        }
                    }
                }

                //std::cout << rMat << std::endl;
                //std::cout << tvec << std::endl;
                //std::cout << rt   << std::endl;

                cv::Mat P(3,4, cv::DataType<double>::type);
                P = cameraMatrix * rt;
                //std::cout << P   << std::endl;
                if ( k == 'c' || k == 'C')
                {
                    std::ofstream outfile;
                    int m0 = i > 0 ? (int)log10(i + 1e-4) + 1: 1;

                    std::string filenametxt = PATH + TXT_PATH       + std::string(8-m0,'0') + std::to_string(i) + ".txt";
                    std::string filenameimg = PATH + VISUALIZE_PATH + std::string(8-m0,'0') + std::to_string(i) + ".jpg";
                    std::cout << "Generating " << filenametxt << " ..." << std::endl;
                    std::cout << "Generating " << filenameimg << " ..." << std::endl;
                    outfile.open(filenametxt, std::ios_base::app);

                    //std::cout << P << std::endl;
                    outfile << "CONTOUR\n"; 
                    outfile << std::to_string(P.at<double>(0,0)) + " " + std::to_string(P.at<double>(0,1)) + " " + std::to_string(P.at<double>(0,2)) + " " + std::to_string(P.at<double>(0,3)) + "\n";
                    outfile << std::to_string(P.at<double>(1,0)) + " " + std::to_string(P.at<double>(1,1)) + " " + std::to_string(P.at<double>(1,2)) + " " + std::to_string(P.at<double>(1,3)) + "\n";;
                    outfile << std::to_string(P.at<double>(2,0)) + " " + std::to_string(P.at<double>(2,1)) + " " + std::to_string(P.at<double>(2,2)) + " " + std::to_string(P.at<double>(2,3)) + "\n";;
                    outfile.close();

                    cv::imwrite(filenameimg, frame);

                    
                    i++;

                    std::ofstream optionstxt;
                    std::string filenameOptions = PATH + "option.txt";
                    optionstxt.open(filenameOptions, std::ofstream::out | std::ofstream::trunc);

                    optionstxt << "level 1\n";
                    optionstxt << "csize 2\n";
                    optionstxt << "threshold 0.7\n";
                    optionstxt << "wsize 7\n";
                    optionstxt << "minImageNum 3\n";
                    optionstxt << "CPU 4\n";
                    optionstxt << "setEdge 0\n";
                    optionstxt << "useBound 0\n";
                    optionstxt << "useVisData 1\n";
                    optionstxt << "sequence -1\n";
                    optionstxt << "timages -1 0 " + std::to_string(i) + "\n";
                    optionstxt << "oimages -3\n";
                } 
            }        
        }

        cv::imshow("Template", Template);
        //video.write(Template);
    }

    cap.release();
    video.release();
    cv::destroyAllWindows();
    return 0;
}
