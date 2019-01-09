#include "utils.h"
#include "base.h"

cv::Mat frameChess;
cv::Size sizeFrameChess;

cv::Mat frameSym;
cv::Size sizeFrameSym;

cv::Mat frameAsym;

cv::Mat result_chess;
cv::Mat result_Asym;
cv::Mat result_Sym;

cv::Mat grayChess;
cv::Mat graySym;
cv::Mat grayAsym;

clock_t begin_time_chess;
clock_t begin_time_circleAsym;
clock_t begin_time_circleSym;

float time_avr_chess;
int count_frame_chess;
float time_elapsed_chess;
float time_avr_Asym;
int count_frame_Asym;
float time_elapsed_Asym;
float time_avr_Sym;
int count_frame_Sym;
float time_elapsed_Sym;

cv::Size board_sz;
std::vector<cv::Point2f> patternChess;
std::vector<std::vector<cv::Point2f>> arrayPatternChess;
std::vector<cv::Point3f> objChess;
std::vector<std::vector<cv::Point3f>> arrayObjChess;
float errorChess;

cv::Mat intrinsicChess = cv::Mat(3, 3, CV_32FC1);
cv::Mat distCoeffsChess;
std::vector<cv::Mat> rvecsChess;
std::vector<cv::Mat> tvecsChes;

cv::Size board_sz_sym;
std::vector<cv::Point2f> patternCircleSym;
std::vector<std::vector<cv::Point2f>> arrayPatternCircleSym;
std::vector<cv::Point3f> objCircleSym;
std::vector<std::vector<cv::Point3f>> arrayObjCircleSym;
float errorCiclesSym;

cv::Mat intrinsicCircleSym= cv::Mat(3, 3, CV_32FC1);
cv::Mat distCoeffsCircleSym;
std::vector<cv::Mat> rvecsCircleSym;
std::vector<cv::Mat> tvecsCircleSym;

cv::Size board_sz_asym;
std::vector<cv::Point2f> patternCircleAsym;
std::vector<std::vector<cv::Point2f>> arrayPatternCircleAsym;
std::vector<cv::Point3f> objCircleAsym;
std::vector<std::vector<cv::Point3f>> arrayObjCircleAsym;
float errorCiclesAsym;

cv::Mat intrinsicCircleAsym= cv::Mat(3, 3, CV_32FC1);
cv::Mat distCoeffsCircleAsym;
std::vector<cv::Mat> rvecsCircleAsym;
std::vector<cv::Mat> tvecsCircleAsym;

cv::Mat chessboar(cv::Mat frameChess,clock_t begin_time_chess);
cv::Mat circleAsymmetric(cv::Mat frameAsym,clock_t begin_time_circleAsym);
cv::Mat  circleSymmetric(cv::Mat frameSym,clock_t begin_time_circleSym);

std::vector<cv::Vec3f> circles;

int main(int argc, char** argv)
{
    //std::string filename = argv[1];
    cv::VideoCapture cap1("../files/board.avi");
    cv::VideoCapture cap2("../files/circlesAsym.avi");
    cv::VideoCapture cap3("../files/circlesSym.avi");

    if (!cap1.isOpened())
    {
        std::cout << "Failed to open chessboard video" << std::endl;
        return -1;
    }
    if (!cap2.isOpened())
    {
        std::cout << "Failed to open circles Asymetric video ." << std::endl;
        return -1;
    }
        if (!cap3.isOpened())
    {
        std::cout << "Failed to open circles Symetric video ." << std::endl;
        return -1;
    }

    int numCornersHor = 8;
    int numCornersVer = 6;
    int numSquares = numCornersHor * numCornersVer;
    board_sz = cv::Size(numCornersHor, numCornersVer);
    for(int j=0;j<numSquares;j++)
        objChess.push_back(cv::Point3f(j/numCornersHor, j%numCornersHor, 0.0f));

    int numCornersHor_Sym = 7;
    int numCornersVer_Sym = 5;
    int numCircles = numCornersHor_Sym * numCornersVer_Sym;
    board_sz_sym = cv::Size(numCornersHor_Sym, numCornersVer_Sym);
    for(int j=0;j<numCircles;j++)
        objCircleSym.push_back(cv::Point3f(j/numCornersHor_Sym, j%numCornersHor_Sym, 0.0f));

    int numCornersHor_Asym = 7;
    int numCornersVer_Asym = 5;
    int numCirclesAsym = numCornersHor_Asym * numCornersVer_Asym;
    board_sz_sym = cv::Size(numCornersHor_Asym, numCornersVer_Asym);
    for(int j=0;j<numCirclesAsym;j++)
        objCircleAsym.push_back(cv::Point3f(j/numCornersHor_Asym, j%numCornersHor_Asym, 0.0f));

    for(;;)
    {
        cap1 >> frameChess;
        if(frameChess.empty())
            break;
            
        cap2 >> frameAsym;
        cap3 >> frameSym;

        begin_time_chess = clock();
        begin_time_circleAsym = clock();
        begin_time_circleSym = clock();

        result_chess = chessboar(frameChess,begin_time_chess);
        //result_Asym = circleAsymetric(frameAsym,begin_time_circleAsym);
        result_Sym = circleSymmetric(frameSym,begin_time_circleSym);
        cv::imshow("chess",result_chess);
        //cv::imshow("asy",result_Asym);
        cv::imshow("sym",result_Sym);
        //times 
        //time_elapsed_Asym
        //time_elapsed_Sym
        //time_elapsed_chess
        char k = cv::waitKey(1);
        if ( k == 27) 
            break;
    }

    cap1.release();
    errorChess = cv::calibrateCamera(arrayObjChess, arrayPatternChess, sizeFrameChess,intrinsicChess, distCoeffsChess, rvecsChess, tvecsChes);
    std::cout << "error " << errorChess/arrayObjChess.size() << std::endl;
    std::cout << "Intrinsic " << intrinsicChess << std::endl;
    
    cap2.release();
    errorCiclesSym = cv::calibrateCamera(arrayObjCircleSym, arrayPatternCircleSym, sizeFrameSym,intrinsicCircleSym, distCoeffsCircleSym, rvecsCircleSym, tvecsCircleSym);
    std::cout << "error " << errorCiclesSym/arrayObjCircleSym.size() << std::endl;
    std::cout << "Intrinsic " << intrinsicCircleSym << std::endl;
    cap3.release();
    return 0;
}

cv::Mat chessboar(cv::Mat frameChess,clock_t begin_time_chess){


    cv::cvtColor(frameChess, grayChess, cv::COLOR_BGR2GRAY);
        //cv::GaussianBlur(gray, gray, cv::Size(3,3), 0, 0); 
    bool found = cv::findChessboardCorners(frameChess, board_sz, patternChess,cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);
    if(found)
    {
        cv::cornerSubPix(grayChess, patternChess, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
        cv::drawChessboardCorners(frameChess, board_sz, patternChess, found);
        arrayPatternChess.push_back(patternChess);
        arrayObjChess.push_back(objChess);
        sizeFrameChess = frameChess.size();
    }

    float temp_time_chess = float( clock () - begin_time_chess ) /  CLOCKS_PER_SEC;
    time_avr_chess  += temp_time_chess;
    time_elapsed_chess = time_avr_chess/count_frame_chess;
    count_frame_chess++;
    return frameChess;
}

cv::Mat circleAsymmetric(cv::Mat frameAsym,clock_t begin_time_circleAsym){
    cv::cvtColor(frameAsym, grayAsym, cv::COLOR_BGR2GRAY);
    bool found = findCirclesGrid( frameAsym, board_sz_sym, patternCircleSym);
    if(found){
        //cv::cornerSubPix(graySym, patternCircleSym, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.001));
        cv::drawChessboardCorners( frameSym, board_sz_sym, cv::Mat(patternCircleSym), found );
        arrayPatternCircleSym.push_back(patternCircleSym);
        sizeFrameSym = frameSym.size();
        arrayObjCircleSym.push_back(objCircleSym);
    }
    /*cv::HoughCircles(grayAsym,circles,CV_HOUGH_GRADIENT,1,grayAsym.rows/16,100,30,1,30);

    for(size_t i=0;i < circles.size();i++){
        cv::Vec3i c = circles[i];
        cv::Point center = cv::Point(c[0],c[1]);
        cv::circle(frameAsym,center,1,cvScalar(0,100,100),3,cv::LINE_AA);
        int radius = c[2];
        cv::circle(circles,center,radius,cvScalar(255,0,255),3,cv::LINE_AA);
    }*/
    
    float temp_time_Asym = float( clock () - begin_time_circleAsym ) /  CLOCKS_PER_SEC;
    time_avr_Asym  += temp_time_Asym;
    time_elapsed_Asym = time_avr_chess/count_frame_Asym;
    count_frame_Asym++;
    return frameAsym;
}

cv::Mat  circleSymmetric(cv::Mat frameSym,clock_t begin_time_circleSym){
    cv::cvtColor(frameSym, graySym, cv::COLOR_BGR2GRAY);
    bool found = findCirclesGrid( frameSym, board_sz_sym, patternCircleSym);
    if(found){
        //cv::cornerSubPix(graySym, patternCircleSym, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.001));
        cv::drawChessboardCorners( frameSym, board_sz_sym, cv::Mat(patternCircleSym), found );
        arrayPatternCircleSym.push_back(patternCircleSym);
        sizeFrameSym = frameSym.size();
        arrayObjCircleSym.push_back(objCircleSym);
    }

    float temp_time_Sym = float( clock () - begin_time_circleSym ) /  CLOCKS_PER_SEC;
    time_avr_Sym  += temp_time_Sym;
    time_elapsed_Sym = time_avr_Sym/count_frame_Sym;
    count_frame_Sym++;
    return frameSym;
}