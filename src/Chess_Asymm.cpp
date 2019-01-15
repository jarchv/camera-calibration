#include "utils.h"
#include "base.h"

std::vector<cv::Mat> arrayImageChess;
std::vector<cv::Mat> arrayImageCircleSym;
std::vector<cv::Mat> arrayImageCircleAsym;

cv::Mat frameChess;
cv::Size sizeFrameChess;

cv::Mat frameSym;
cv::Size sizeFrameSym;

cv::Mat frameAsym;
cv::Size sizeFrameAsym;

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
std::vector<cv::Mat> tvecsChess;

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

cv::Size board_sz_Asym;
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


int maxFrame = 30;
int main(int argc, char** argv)
{
    cv::VideoCapture cap1("../files/cam2/chess.avi");
    cv::VideoCapture cap2("../files/cam2/circulos.avi");

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

    int numCornersHor = 8;
    int numCornersVer = 6;
    int numSquares = numCornersHor * numCornersVer;
    board_sz = cv::Size(numCornersHor, numCornersVer);
    for(int j=0;j<numSquares;j++)
        objChess.push_back(cv::Point3f((j/numCornersHor)*48.0, (j%numCornersHor)*48.0, 0.0f));

    int numCornersHor_Sym = 7;
    int numCornersVer_Sym = 5;
    int numCircles = numCornersHor_Sym * numCornersVer_Sym;
    board_sz_sym = cv::Size(numCornersHor_Sym, numCornersVer_Sym);
    float distanceSym = 35;
    for( int i = 0; i < board_sz_sym.height; ++i )
        for( int j = 0; j < board_sz_sym.width; ++j )
           objCircleSym.push_back(cv::Point3f(float( j*distanceSym ), float( i*distanceSym ), 0));

    int numCornersHor_Asym = 4;
    int numCornersVer_Asym = 11;
    int numCirclesAsym = numCornersHor_Asym * numCornersVer_Asym;
    board_sz_Asym = cv::Size(numCornersHor_Asym, numCornersVer_Asym);
 
    float distanceAsym = 18;
    for( int i = 0; i < board_sz_Asym.height; i++ )
        for( int j = 0; j < board_sz_Asym.width; j++ )
            objCircleAsym.push_back(cv::Point3f(float((2*j + i % 2)*distanceAsym), float(i*distanceAsym), 0));

    int countFrame =0 ;
    for(;;)
    {
        cap1 >> frameChess;
        cap2 >> frameAsym;

        begin_time_chess = clock();
        begin_time_circleAsym = clock();
        begin_time_circleSym = clock();

        if(frameChess.empty() == false)
        {
            result_chess = chessboar(frameChess,begin_time_chess);   
        }
            
        if(frameAsym.empty() == false)
        {
            result_Asym = circleAsymmetric(frameAsym,begin_time_circleAsym);
        }
            
        if(frameChess.empty() && frameAsym.empty()){
            break;
        }

        countFrame++;
        char k = cv::waitKey(1);
        if ( k == 27) 
            break;
        
    }

    cap1.release();
    std::cout << "Result Chessboard" << std::endl;
    errorChess = cv::calibrateCamera(arrayObjChess, arrayPatternChess, sizeFrameChess,intrinsicChess, distCoeffsChess, rvecsChess, tvecsChess);
    std::cout << "error " << errorChess << std::endl;
    std::cout << "Intrinsic " << intrinsicChess << std::endl;
    
    cap2.release();
    errorCiclesAsym = cv::calibrateCamera(arrayObjCircleAsym, arrayPatternCircleAsym, sizeFrameAsym,intrinsicCircleAsym, distCoeffsCircleAsym, rvecsCircleAsym, tvecsCircleAsym,CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);
    std::cout << "error " << errorCiclesAsym<< std::endl;
    std::cout << "Intrinsic " << intrinsicCircleAsym << std::endl;
    cv::imshow("intrinsic",intrinsicCircleAsym);
    return 0;
}

cv::Mat chessboar(cv::Mat frameChess,clock_t begin_time_chess){


    cv::cvtColor(frameChess, grayChess, cv::COLOR_BGR2GRAY);
    bool found = cv::findChessboardCorners(frameChess, board_sz, patternChess,cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE + cv::CALIB_CB_FAST_CHECK);
    if(found && count_frame_chess % 25 == 0)
    {
        cv::cornerSubPix(grayChess, patternChess, cv::Size(11, 11), cv::Size(-1, -1), cv::TermCriteria(CV_TERMCRIT_EPS | CV_TERMCRIT_ITER, 30, 0.1));
        cv::drawChessboardCorners(frameChess, board_sz, patternChess, found);
        arrayPatternChess.push_back(patternChess);
        arrayObjChess.push_back(objChess);
        sizeFrameChess = frameChess.size();
        cv::imshow("chess",frameChess);
        std::cout << count_frame_chess << std::endl;
    }

    float temp_time_chess = float( clock () - begin_time_chess ) /  CLOCKS_PER_SEC;
    time_avr_chess  += temp_time_chess;
    time_elapsed_chess = time_avr_chess/count_frame_chess;
    count_frame_chess++;
    return frameChess;
}

cv::Mat circleAsymmetric(cv::Mat frameAsym,clock_t begin_time_circleAsym){
    cv::cvtColor(frameAsym, grayAsym, cv::COLOR_BGR2GRAY);
    bool found = findCirclesGrid( frameAsym, board_sz_Asym, patternCircleAsym,cv::CALIB_CB_ASYMMETRIC_GRID);
    if(found && count_frame_Asym % 25 == 0){

        cv::drawChessboardCorners( frameAsym, board_sz_Asym, cv::Mat(patternCircleAsym), found );
        arrayPatternCircleAsym.push_back(patternCircleAsym);
        sizeFrameAsym = frameAsym.size();
        cv::imshow("asy",frameAsym);
        arrayObjCircleAsym.push_back(objCircleAsym);
        std::cout << count_frame_Asym << std::endl;
    }

    float temp_time_Asym = float( clock () - begin_time_circleAsym ) /  CLOCKS_PER_SEC;
    time_avr_Asym  += temp_time_Asym;
    time_elapsed_Asym = time_avr_chess/count_frame_Asym;
    count_frame_Asym++;
    return frameAsym;
}

cv::Mat  circleSymmetric(cv::Mat frameSym,clock_t begin_time_circleSym){
    cv::cvtColor(frameSym, graySym, cv::COLOR_BGR2GRAY);
    bool found = findCirclesGrid( frameSym, board_sz_sym, patternCircleSym);
    if(found && count_frame_Sym % 12 == 0){

        cv::drawChessboardCorners( frameSym, board_sz_sym, cv::Mat(patternCircleSym), found );
        arrayPatternCircleSym.push_back(patternCircleSym);
        sizeFrameSym = frameSym.size();
        arrayObjCircleSym.push_back(objCircleSym);
        std::cout << count_frame_Sym << std::endl;
    }

    float temp_time_Sym = float( clock () - begin_time_circleSym ) /  CLOCKS_PER_SEC;
    time_avr_Sym  += temp_time_Sym;
    time_elapsed_Sym = time_avr_Sym/count_frame_Sym;
    count_frame_Sym++;
    return frameSym;
}