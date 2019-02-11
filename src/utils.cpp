#include "utils.h"
#include <algorithm>

#include <opencv2/calib3d/calib3d.hpp>

#define PI 3.14159265

int ITERATIONS = 15;

void selfCapture(cv::Mat& src, 
                std::vector<cv::Point> SortedPoints,
                cv::Mat toModel, 
                std::vector<cv::Mat>& imgToCalib, 
                std::vector<std::vector<cv::Point2f>>& imagePoints,
                std::vector<cv::Point3f> PatternPointsPositions,
                cv::Size BoardSize,
                std::vector<double>& ThetaArray,
                std::vector<double>& PhiXArray,
                std::vector<double>& PhiYArray,
                bool FORCE)
{
    float BIAS = 200.0;

    //std::cout << "thetas Size : " << ThetaArray.size() << std::endl;
    if (SortedPoints.size() == (BoardSize.width * BoardSize.height))
    {
        std::vector<cv::Point2f> ObjectPointsModel2D;
        for(int i = 0; i < PatternPointsPositions.size(); i++)
        {
            ObjectPointsModel2D.push_back(cv::Point2f(PatternPointsPositions[i].x + BIAS, PatternPointsPositions[i].y + BIAS/2.0));
        }

        std::vector<cv::Point2f> imagePointsModel2D;
        for(int i = 0; i < SortedPoints.size(); i++)
        {
            imagePointsModel2D.push_back(cv::Point2f(SortedPoints[i].x, SortedPoints[i].y));
        }

        cv::Mat H = cv::findHomography(imagePointsModel2D, ObjectPointsModel2D,CV_RANSAC);
        if (H.cols == 3 && H.rows == 3)
        {

            double norm = sqrt(H.at<double>(0,0)*H.at<double>(0,0) +
                        H.at<double>(1,0)*H.at<double>(1,0) +
                        H.at<double>(2,0)*H.at<double>(2,0));
            H /= norm;
            cv::Mat c1  = H.col(0);
            cv::Mat c2  = H.col(1);
            cv::Mat c3 = c1.cross(c2);
            cv::Mat tvec = H.col(2);
            cv::Mat R(3, 3, CV_64F);

            double modZ_dir = 0.0;
            for (int ir = 0; ir < 3; ir++)
            {
                R.at<double>(ir,0) = c1.at<double>(ir,0);
                R.at<double>(ir,1) = c2.at<double>(ir,0);
                R.at<double>(ir,2) = c3.at<double>(ir,0);

                modZ_dir += (c3.at<double>(ir,0) * c3.at<double>(ir,0));
            }
                        
            double theta = atan (R.at<double>(1,0)) * 180.0 / PI;
            
            double cos1 = 500.0*R.at<double>(0,2)/sqrt(modZ_dir);
            double cos2 = 500.0*R.at<double>(1,2)/sqrt(modZ_dir);
            
            if (cos1 < -1)
                cos1 = -1;
            else if (cos1 > 1)
                cos1 = 1;

            if (cos2 < -1)
                cos2 = -1;
            else if (cos2 > 1)
                cos2 = 1;


            double phiX  = acos (cos1) * 180.0 / PI;
            double phiY  = acos (cos2) * 180.0 / PI;

            bool NEW_ANGLE = true;

            if(tvec.at<double>(2,0) > 0.5 && abs(theta) > 25)
            {
                if (ThetaArray.size() == 0)
                {
                    ThetaArray.push_back(theta);
                    PhiXArray.push_back(phiX);
                    PhiYArray.push_back(phiY);
                }   
                else
                {
                    for(int it=0; it < ThetaArray.size(); it++)
                    {
                        if (abs(ThetaArray[it] - theta) < 25 && abs(PhiXArray[it] - phiX) < 15 && abs(PhiYArray[it] - phiY) < 15)
                        {
                            NEW_ANGLE = false;
                            break;
                        }
                    }

                    if (NEW_ANGLE == true)
                    {
                        ThetaArray.push_back(theta);
                        PhiXArray.push_back(phiX);
                        PhiYArray.push_back(phiY);
                    }
                }
                
            }

            else
            {
                NEW_ANGLE = false;
            }

            if (NEW_ANGLE == true || FORCE == true)
            {
                std::vector<cv::Point2f> pointBuf(BoardSize.width * BoardSize.height);
                for (int i = 0; i < SortedPoints.size(); i++)
                {
                    pointBuf[i] = cv::Point2f(SortedPoints[i].x, SortedPoints[i].y);          
                }

                imagePoints.push_back(pointBuf);
                std::cout << "Capturing image | current size = " << imagePoints.size() << std::endl;
                imgToCalib.push_back(toModel); 

                if (FORCE == true && NEW_ANGLE == false)
                {
                    ThetaArray.push_back(theta);
                    PhiXArray.push_back(phiX);
                    PhiYArray.push_back(phiY);                    
                }
            }
            /*
            std::cout << "==================" << std::endl;
            std::cout << R.at<double>(0,0) << " " << R.at<double>(0,1) << " " <<R.at<double>(0,2) << std::endl;
            std::cout << R.at<double>(1,0) << " " << R.at<double>(1,1) << " " <<R.at<double>(1,2) << std::endl;
            std::cout << R.at<double>(2,0) << " " << R.at<double>(2,1) << " " <<R.at<double>(2,2) << std::endl;
            std::cout << "==================" << std::endl;
            */
        }
    }
}
void CreateObject3D(cv::Mat& view, std::vector<cv::Point> ObjectPointsProjected2Image)
{
    cv::line(view, ObjectPointsProjected2Image[0], ObjectPointsProjected2Image[4], cv::Scalar(255,0,255), 4, 8);
    cv::line(view, ObjectPointsProjected2Image[8], ObjectPointsProjected2Image[5], cv::Scalar(255,255,0), 4, 8);
    cv::line(view, ObjectPointsProjected2Image[9], ObjectPointsProjected2Image[6], cv::Scalar(0,255,255), 4, 8);
    cv::line(view, ObjectPointsProjected2Image[10], ObjectPointsProjected2Image[7], cv::Scalar(255,0,255), 4, 8);

    cv::line(view, ObjectPointsProjected2Image[4], ObjectPointsProjected2Image[5], cv::Scalar(0,255,0), 4, 8);
    cv::line(view, ObjectPointsProjected2Image[5], ObjectPointsProjected2Image[7], cv::Scalar(255,0,0), 4, 8);
    cv::line(view, ObjectPointsProjected2Image[6], ObjectPointsProjected2Image[7], cv::Scalar(0,0,255), 4, 8);
    cv::line(view, ObjectPointsProjected2Image[6], ObjectPointsProjected2Image[4], cv::Scalar(255,255,0), 4, 8);

    cv::line(view, ObjectPointsProjected2Image[0 ], ObjectPointsProjected2Image[ 8], cv::Scalar(255,0,0), 4, 8);
    cv::line(view, ObjectPointsProjected2Image[0 ], ObjectPointsProjected2Image[ 9], cv::Scalar(0,125,255), 4, 8);
    cv::line(view, ObjectPointsProjected2Image[10], ObjectPointsProjected2Image[ 8], cv::Scalar(0,255,0), 4, 8);
    cv::line(view, ObjectPointsProjected2Image[10], ObjectPointsProjected2Image[ 9], cv::Scalar(255,0,0), 4, 8);   
}
void Mat2Mat(cv::Mat& src, cv::Mat& dst, int x0, int y0)
{
    for(int i = x0; i < x0 + src.rows; i++)
    {
        for(int j = y0; j < y0 + src.cols; j++)
        {
            dst.at<cv::Vec3b>(i, j) = src.at<cv::Vec3b>(i-x0, j-y0);
        }
    }
}

void drawLines(cv::Mat& img, std::vector<cv::Point> SortedPoints)
{
    cv::Scalar color = cv::Scalar( 255, 250, 50);
    for (int ip = 0; ip < SortedPoints.size(); ip++)
    {
        cv::circle(img, cv::Point(SortedPoints[ip].x, SortedPoints[ip].y), 2, cv::Scalar(0,0,255), 2, 8);  
        putText(img,std::to_string(ip+1),cv::Point(SortedPoints[ip].x, SortedPoints[ip].y),cv::FONT_ITALIC,0.8,color,2);           
    }
}
void thresholdIntegral(cv::Mat &inputMat, cv::Mat &outputMat, double T)
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

    //int S = MAX(nRows, nCols)/8;
    int S = MAX(nRows, nCols)/8;
    //double T = 0.08;
    //double T = 0.15;
    // perform thresholding
    int s2 = S/2;
    int x1, y1, x2, y2, count, sum;

    // CV_Assert(sizeof(int) == 4);
    int *p_y1, *p_y2;
    uchar *p_inputMat, *p_outputMat;

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
            sum = p_y2[x2] - p_y1[x2] - p_y2[x1] + p_y1[x1];

            if ((int)(p_inputMat[j] * count) < (int)(sum*(1.0-T)))
                p_outputMat[j] = 255;
            else
                p_outputMat[j] = 0;
        }
    }
}



int PointsInside(std::vector<cv::Point>& P, std::vector<cv::Point>& pIns,float m, float b, int np)
{
    int     count = 0;
    float   y_hat;
    float   K = 10.0;

    for (int i = 0; i < P.size(); i++)
    {
        y_hat = (float)P[i].x * m + b;  
        if (abs(y_hat - (float)P[i].y) < K)
        {   
            count++;
            pIns.push_back(cv::Point(P[i].x, P[i].y));
        }           
    }

    if (count == np)
    {
        for (int i = 0; i < pIns.size(); i++)
        {
            if (std::find(P.begin(),P.end(),cv::Point(pIns[i].x, pIns[i].y)) != P.end())
                P.erase(std::find(P.begin(),P.end(),cv::Point(pIns[i].x, pIns[i].y)));           
        }
    }
    return count;
}

std::vector<std::vector<cv::Point>> Points2RectOfPoints(std::vector<cv::Point> allPoints2D, cv::Size BoardSize)
{
    std::vector<cv::Point> tempPoints(allPoints2D.size());

    for (int i = 0; i < allPoints2D.size(); i++)
    {
        tempPoints[i].x  = allPoints2D[i].x;
        tempPoints[i].y  = allPoints2D[i].y;
    }

    std::vector<cv::Point> lhor(2);

    float m;
    float b;

    int npoints;

    std::vector<std::vector<cv::Point>> RectOfPoinst;
    std::vector<cv::Point> pIns;

    
    for (int i = 0; i < allPoints2D.size(); i++)
    {
        for (int j = 0; j < allPoints2D.size(); j++)
        {
            if (i != j)
            {
                m       = (float)(allPoints2D[i].y - allPoints2D[j].y) / ((float)(allPoints2D[i].x - allPoints2D[j].x));
                b       = (float)allPoints2D[i].y - allPoints2D[i].x * m;
                
                pIns.clear();

                npoints = PointsInside(tempPoints, pIns, m, b, BoardSize.width);
                if (npoints == BoardSize.width)
                {
                    RectOfPoinst.push_back(pIns);
                }
            }
        }
    }
    return RectOfPoinst;
}

struct myclass {
    bool operator() (cv::Point pt1, cv::Point pt2) { return (pt1.x < pt2.x);}
} myobject;

void SortingPoints(std::vector<cv::Point>  tempCnts, std::vector<cv::Point>& nextCenters, cv::Size BoardSize)
{
    if (tempCnts.size() == (BoardSize.width * BoardSize.height))
    {
        std::vector<std::vector<cv::Point>> RofP = Points2RectOfPoints(tempCnts,BoardSize);
        std::vector<std::vector<int>> Ypos;

        std::vector<cv::Point> minYVec;
        std::vector<int> Y;
        for (int ri = 0; ri < RofP.size(); ri++)
        {
            Y.clear();
            for(int iY = 0; iY < RofP[ri].size(); iY++)
            {
                Y.push_back(RofP[ri][iY].y);
            }

            int maxY = *max_element(Y.begin(), Y.end());
            minYVec.push_back(cv::Point(ri, maxY));
        }

        std::vector<std::vector<cv::Point>> SortedRofP;
        std::vector<cv::Point> SortedR;

        while (minYVec.size() != 0)
        {
            SortedR.clear();

            int Global_minY     = minYVec[0].y;
            int Global_minY_Pos = minYVec[0].x;

            if (minYVec.size() > 1)
            {
                for (int iY = 1; iY < minYVec.size(); iY++)
                {
                    if (minYVec[iY].y < Global_minY)
                    {
                        Global_minY     = minYVec[iY].y;
                        Global_minY_Pos = minYVec[iY].x;
                    }
                }  

                for (int iP = 0; iP < RofP[Global_minY_Pos].size(); iP++)
                {
                    SortedR.push_back(cv::Point(RofP[Global_minY_Pos][iP].x, RofP[Global_minY_Pos][iP].y));
                }
                SortedRofP.push_back(SortedR);

                if (std::find(minYVec.begin(),minYVec.end(),cv::Point(minYVec[Global_minY_Pos].x, minYVec[Global_minY_Pos].y)) != minYVec.end())
                    minYVec.erase(std::find(minYVec.begin(),minYVec.end(),cv::Point(minYVec[Global_minY_Pos].x, minYVec[Global_minY_Pos].y)));     

            } 
            else
            {
                for (int iP = 0; iP < RofP[0].size(); iP++)
                {
                    SortedR.push_back(cv::Point(RofP[0][iP].x, RofP[0][iP].y));
                }
                SortedRofP.push_back(SortedR);
                minYVec.erase(minYVec.begin(),minYVec.begin());
                break;               
            }                  
        }

        for (int ir = 0; ir < SortedRofP.size(); ir ++)
        {
            std::sort(SortedRofP[ir].begin(), SortedRofP[ir].end(), myobject);
        }

        nextCenters.clear();
        for (int ir = 0; ir < SortedRofP.size(); ir ++)
        {
            for (int iP = 0; iP < SortedRofP[ir].size(); iP++)
            {
                nextCenters.push_back(SortedRofP[SortedRofP.size() - ir - 1][iP]);
            }
        }
    }
    else 
    {
        nextCenters.clear();
    }
}
bool trancking(  std::vector<cv::Point>  tempCnts, 
                std::vector<cv::Point>  prevCenters,
                std::vector<cv::Point>& nextCenters,
                cv::Size BoardSize,
                int countFrame )
{
    int centerx;
    int centery;
    int dist_centers;
    int countc      = 0;
    
    int center_jump = 0;

    float minc;
    int   Nrst;

    cv::Scalar color = cv::Scalar( 255, 250, 50);

    bool jump = false;

    if (countFrame == 0)
    {
        return true;
    }
    else
    {
        int nextX;
        int nextY;

        if (prevCenters.size() != (BoardSize.height * BoardSize.width))
        {
            return true;
        }
        for (int i = 0; i < prevCenters.size(); i++)
        {
            centerx = prevCenters[i].x;
            centery = prevCenters[i].y;

            minc    = 1e5;

            for(int iT =  0; iT < tempCnts.size(); iT++){
                dist_centers = sqrt((centerx - tempCnts[iT].x)*(centerx - tempCnts[iT].x) + 
                                    (centery - tempCnts[iT].y)*(centery - tempCnts[iT].y));
                if(dist_centers < minc){
                    minc  = dist_centers;
                    Nrst  = iT;
                    nextX = tempCnts[iT].x;
                    nextY = tempCnts[iT].y;
                }
            }    

            if (minc < 10)
            {
                nextCenters[i] = cv::Point(nextX, nextY);
            }     

            else 
            {
                return true;
            }
        }

        return false;
    }
    /*
    if(countFrame > 0)
    {
        return true;
    }

    return center_jump;
    */
}

std::vector<cv::Point> findConcentricCenters( std::vector<cv::RotatedRect> minRect,
                            std::vector<cv::Point> centers,
                            std::vector<float> diag,
                            int contours_filtered)
{
    std::vector<cv::Point> tempCnts;
    cv::Point2f rect_points[4];
    int centerx;
    int centery;
    float diagtemp;
    float errormaxDiam = 3.5;
    float errormax = 3; 
    float dist;
    

    for( int i = 0; i< contours_filtered; i++ )
    {
        
        minRect[i].points( rect_points );
        centerx = 0.5*(rect_points[0].x + rect_points[2].x);
        centery = 0.5*(rect_points[0].y + rect_points[2].y);
        centers[i] = cv::Point(centerx,centery); 
        diag[i] =  sqrt((rect_points[2].x - rect_points[0].x)*(rect_points[2].x - rect_points[0].x)+(rect_points[2].y - rect_points[0].y)*(rect_points[2].y - rect_points[0].y));
    }

    for(int p = 0; p < contours_filtered; p++){
        minRect[p].points( rect_points );
        centerx = 0.5*(rect_points[0].x + rect_points[2].x);
        centery = 0.5*(rect_points[0].y + rect_points[2].y);
        diagtemp = sqrt(pow((rect_points[2].x - rect_points[0].x),2)+pow((rect_points[2].y - rect_points[0].y),2));

        for(int k = 0; k < contours_filtered; k++)
        {
            if(k != p){
                dist = sqrt((centerx - centers[k].x)*(centerx - centers[k].x) + (centery - centers[k].y)*(centery - centers[k].y));
                if((dist <= errormax) && (diag[k] - diagtemp)  > errormaxDiam)
                {   
                    tempCnts.push_back(cv::Point(0.5*(centerx + centers[k].x), 0.5*(centery + centers[k].y)));
                    break;
                }
            }
        }
    }

    return tempCnts;   
}

cv::Mat findCenters(cv::Mat frame, 
                    cv::Mat gray, 
                    cv::Mat& bin, 
                    cv::Mat& contours_draw,
                    std::vector<std::vector<cv::Point>>& contours,
                    int& countFrame,
                    std::vector<cv::Point>& OldPoints,
                    cv::Size BoardSize,
                    double T)
{
    countFrame++;
    int prevcenterx = 0;
    int prevcentery = 0;
    cv::Point prevCenter = cv::Point(0,0);
    cv::Mat prev;
    int maxdistance = 0;

    std::vector<cv::Vec4i> hierarchy;
    std::vector<cv::Point> newPoints(BoardSize.width * BoardSize.height);
    std::vector<cv::Point> tempCnts;

    cv::Mat result;

    cv::Mat contours_input;
    frame.copyTo(result);
    int cb              = 0;

    bin = cv::Mat::zeros(gray.size(), CV_8UC1);
    thresholdIntegral(gray,bin, T);

    bin.copyTo(contours_draw);
    cv::cvtColor(contours_draw, contours_draw, CV_GRAY2RGB);
    bin.copyTo(contours_input);

    cv::findContours(contours_input, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE, prevCenter);


    std::vector<cv::Point> centers(contours.size());
    std::vector<float>     diag(contours.size());

    std::vector<cv::RotatedRect> minRect( contours.size() );
    int contours_filtered = 0;
    
    for( int i = 0; i < contours.size(); i++ )
    {
        if(contours[i].size() > 10  && contours[i].size() < 280)
        {
            minRect[contours_filtered]    = cv::minAreaRect( cv::Mat(contours[i]) );
            contours_filtered++;
        }
    }

    cv::Scalar color;
    int   centerx;
    int   centery;
    float minc;
    float dist_centers;

    color = cv::Scalar( 255, 250, 50);
    int countc=0;

    tempCnts    = findConcentricCenters(minRect, centers, diag, contours_filtered);

    for(int i=0; i < tempCnts.size(); i++)
    {
        cv::circle(result, cv::Point(tempCnts[i].x, tempCnts[i].y), 2, cv::Scalar(0,255,0), 2, 8);  
    }

    bool Cjump = trancking( tempCnts, 
                            OldPoints,
                            newPoints,
                            BoardSize,
                            countFrame);   
    
    if (Cjump == true)
    {
        if (tempCnts.size() < (BoardSize.width * BoardSize.height))
        {
            OldPoints.clear();
            return result;
        }
        std::vector<cv::Point> newPoints;

        SortingPoints(tempCnts, newPoints, BoardSize);

        if (newPoints.size() == (BoardSize.width * BoardSize.height))
        {
            OldPoints = newPoints;
        }

        else{
            OldPoints.clear();
        }
    }

    else {
        OldPoints = newPoints;
    }

    
    
    return result;
}


/*
* CALIBRATION
* ===============================================================================================================
*/

static double computeReprojectionErrors( const std::vector<std::vector<cv::Point3f> >& objectPoints,
                                         const std::vector<std::vector<cv::Point2f> >& imagePoints,
                                         const std::vector<cv::Mat>& rvecs, const std::vector<cv::Mat>& tvecs,
                                         const cv::Mat& cameraMatrix , const cv::Mat& distCoeffs,
                                         std::vector<float>& perViewErrors)
{
    std::vector<cv::Point2f> imagePoints2;
    int i, totalPoints = 0;
    double totalErr = 0, err;
    perViewErrors.resize(objectPoints.size());

    for( i = 0; i < (int)objectPoints.size(); ++i )
    {
        projectPoints( cv::Mat(objectPoints[i]), rvecs[i], tvecs[i], cameraMatrix,
                       distCoeffs, imagePoints2);
        err = cv::norm(cv::Mat(imagePoints[i]), cv::Mat(imagePoints2), CV_L2);

        int n = (int)objectPoints[i].size();
        perViewErrors[i] = (float) std::sqrt(err*err/n);
        totalErr        += err*err;
        totalPoints     += n;
    }

    return std::sqrt(totalErr/totalPoints);
}


float IterativeRefinement(std::vector<cv::Mat> imgsToCalib,
                         std::vector<cv::Point3f> objectPoints,
                         std::vector<std::vector<cv::Point2f>>& imagePoints,
                         cv::Mat& cameraMatrix , cv::Mat& distCoeffs,
                         std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs,
                         cv::Size BoardSize,
                         double& getAvr,
                         int itrNumber)
{
    cv::Mat dst;
    float BIAS = 200.0;

    std::vector<std::vector<cv::Point3f>> NewObjectPointsModel(imagePoints.size());
    std::vector<std::vector<cv::Point2f>> imagePointsReProyected(imagePoints.size());

    cv::Mat temp;
    float D = 45.0;
    for(int i = 0; i < imagePoints.size(); i++)
    {
        dst  = imgsToCalib[i].clone();
        temp = dst.clone();

        std::vector<cv::Point2f> ObjectPointsModel2D;
        for(int j = 0; j < objectPoints.size(); j++)
        {
            ObjectPointsModel2D.push_back(cv::Point2f(objectPoints[j].x + BIAS, D * (BoardSize.height - 1) - objectPoints[j].y + BIAS/2));
        }

        std::vector<cv::Point2f> imagePointsModel2D;

        for(int j = 0; j < imagePoints[i].size(); j++)
        {
            imagePointsModel2D.push_back(cv::Point2f(imagePoints[i][j].x, imagePoints[i][j].y));
        }

        cv::Mat M = cv::findHomography(imagePointsModel2D, ObjectPointsModel2D);

        if (M.cols != 3 || M.rows != 3)
        {
            NewObjectPointsModel[i] = objectPoints;
            cv::projectPoints(NewObjectPointsModel[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePointsReProyected[i]);
            continue;
        }

        cv::undistort(temp, dst, cameraMatrix, distCoeffs);

        warpPerspective(temp, dst, M, dst.size());
        //cv::imshow("dst",dst);        

        std::vector<cv::Point> SortedPoints2;
        cv::Mat gray2;
        cv::Mat bin2;
        cv::Mat contours_draw2;
        std::vector<std::vector<cv::Point>> contours2;
        
        int c = 0;

        cv::Rect myROI(BIAS - D * 0.5, BIAS/2 - D*0.5 , D * (BoardSize.width), D * (BoardSize.height));
        cv::Mat croppedRef(dst, myROI);
        
        //cv::imshow("croppedRef",croppedRef);  
        cv::cvtColor(croppedRef, gray2, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray2, gray2, cv::Size(5,5), 0, 0);  

        cv::Mat result2 = findCenters(croppedRef, gray2, bin2, contours_draw2, contours2, c, SortedPoints2, BoardSize, 0.08);


        if (SortedPoints2.size() == (BoardSize.width * BoardSize.height))
        {
            drawLines(result2, SortedPoints2);
            std::vector<cv::Point3f> FimgPoint; 

            int Xl = (int)SortedPoints2[0].x;
            int Yl = (int)SortedPoints2[15].y;
            
            //cv::Rect myROI(Xl - D * 0.5, Yl - D*0.5 , D * (BoardSize.width), D * (BoardSize.height));
            //cv::Mat croppedRef(result2, myROI);
            for(int j = 0; j < SortedPoints2.size(); j++)
            {
                FimgPoint.push_back(cv::Point3f(SortedPoints2[j].x - D*0.5 , D * (BoardSize.height - 1) - (SortedPoints2[j].y - D*0.5), 0.0));
            }
            NewObjectPointsModel[i] = FimgPoint;
            //cv::flip(croppedRef,croppedRef,0);
            cv::imshow("crop", result2);        
            cv::waitKey(30); 
        }
        else{
            std::vector<cv::Point3f> FimgPoint; 
            for(int j = 0; j < objectPoints.size(); j++)
            {
                FimgPoint.push_back(cv::Point3f(objectPoints[j].x, objectPoints[j].y, 0.0));
            }
            NewObjectPointsModel[i] = FimgPoint;
            cv::imshow("crop", result2);        
            cv::waitKey(30); 
        }
        cv::projectPoints(NewObjectPointsModel[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePointsReProyected[i]);
    }
    cv::destroyAllWindows();

    for (int i = 0; i < imagePointsReProyected.size(); i++)
    {
        //std::cout << NewObjectPointsModel[i] << std::endl;
        //float a       = 9.0/8.0;
        //float alpha   = 0.951.068/float(itrNumber+1);
        //if (alpha >= 1)
        //    alpha = 0.9999;

        //alpha = 0.99;
        //float alpha   = 9.0 / (9.0 + exp(-(float)itrNumber/(float(ITERATIONS))));
        float alpha   = 0.95;
        float n_alpha = 1.0 - alpha;
        
        //std::cout << "alpha : " << alpha << std::endl;
        for (int j = 0; j < imagePointsReProyected[i].size(); j++)
        {
            imagePoints[i][j].x = imagePoints[i][j].x * alpha + imagePointsReProyected[i][j].x*n_alpha;
            imagePoints[i][j].y = imagePoints[i][j].y * alpha + imagePointsReProyected[i][j].y*n_alpha;

            imagePointsReProyected[i][j].x = imagePoints[i][j].x;
            imagePointsReProyected[i][j].y = imagePoints[i][j].y;
            //imagePointsReProyected[i][j].x = (imagePoints[i][j].x)*0.8 + (imagePointsReProyected[i][j].x)*0.2;
            //imagePointsReProyected[i][j].y = (imagePoints[i][j].y)*0.8 + (imagePointsReProyected[i][j].y)*0.2;
        //    std::cout << "[i = " << i << "] = " << imagePointsReProyected[i][j] << "  <> "<< imagePoints[i][j] << std::endl;
        }
    }
    
    std::vector<float> projectErrors;
            

    double totalAvgErr = 0;

    std::vector<std::vector<cv::Point3f>> objectPointsNew(1);

    calcPointPosition(objectPointsNew[0], BoardSize);
    objectPointsNew.resize(imagePoints.size(),objectPointsNew[0]);

    float rms = calibrateCamera(objectPointsNew, 
                                            imagePointsReProyected, 
                                            temp.size(), 
                                            cameraMatrix,
                                            distCoeffs, 
                                            rvecs, 
                                            tvecs, 
                                            0);

    std::cout << "Re-projection error reported by calibrateCamera =  "<< rms << std::endl;

    //totalAvgErr = computeReprojectionErrors(objectPointsNew, imagePointsReProyected,
    //                                         rvecs, tvecs, cameraMatrix, distCoeffs, projectErrors);
    //std::cout << "\nAvg_Reprojection_Error = " << totalAvgErr << std::endl;
    getAvr = rms;
    return rms;
}


void calcPointPosition(std::vector<cv::Point3f>& corners, cv::Size BoardSize)
{
    corners.clear();
    float squareSize = 45;
    
    for (int i = 0; i < BoardSize.height; i++)
    {
        for (int j = 0; j < BoardSize.width; j++)
        {
            corners.push_back(cv::Point3f(float( j*squareSize ), float( i*squareSize ), 0));
        }
    }
}
bool Calibration(cv::Size imgSize, 
                cv::Mat& cameraMatrix, 
                cv::Mat& distCoeffs,
                std::vector<std::vector<cv::Point2f>> imagePoints,
                std::vector<cv::Mat>& rvecs,
                std::vector<cv::Mat>& tvecs,
                std::vector<float>& projectErrors,
                double& totalAvgErr,
                double& avr,
                cv::Size BoardSize,
                std::vector<cv::Point3f>& newObjectPoints)
{
    cameraMatrix = cv::Mat::eye(3, 3, CV_64F);

    cameraMatrix.at<double>(0,0) = 3.0/4.0;
    distCoeffs = cv::Mat::zeros(8, 1, CV_64F);

    std::vector<std::vector<cv::Point3f>> objectPoints(1);

    calcPointPosition(objectPoints[0], BoardSize);
    objectPoints.resize(imagePoints.size(),objectPoints[0]);

    newObjectPoints = objectPoints[0];

    float rms = calibrateCamera(objectPoints, 
                                    imagePoints, 
                                    imgSize, 
                                    cameraMatrix,
                                    distCoeffs, 
                                    rvecs, 
                                    tvecs, 
                                    CV_CALIB_FIX_K4|CV_CALIB_FIX_K5);

    avr = rms;

    std::cout << "\nRe-projection error reported by calibrateCamera =  "<< rms << std::endl;

    bool ok = cv::checkRange(cameraMatrix) && cv::checkRange(distCoeffs);
    
    totalAvgErr = computeReprojectionErrors(objectPoints, imagePoints,
                                             rvecs, tvecs, cameraMatrix, distCoeffs, projectErrors);
    
    //std::cout << "\nAvg_Reprojection_Error = " << totalAvgErr << std::endl;
    //std::cout << "\nCalibration Matrix:  \n"<<std::endl;

    for (int im = 0; im < 3; im++)
    {
        for (int jm = 0; jm < 3; jm++)
        {
            std::cout << cameraMatrix.at<double>(im,jm) << " ";
        }
        std::cout << std::endl;
    }
    
    return ok;
}

bool GetParams( std::vector<cv::Mat> imgsToCalib,
                cv::Size imgSize, 
                cv::Mat& cameraMatrix, 
                cv::Mat& distCoeffs,
                std::vector<std::vector<cv::Point2f>> imagePoints,
                double& avr,
                cv::Size BoardSize,
                std::vector<cv::Point3f>& newObjectPoints)
{
    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;

    std::vector<float> projectErrors;
    

    double totalAvgErr = 0;

    bool res = Calibration( imgSize,
                            cameraMatrix,
                            distCoeffs,
                            imagePoints,
                            rvecs,
                            tvecs,
                            projectErrors,
                            totalAvgErr,
                            avr,
                            BoardSize,
                            newObjectPoints);

    float prevError = 1000.0;
    float nextError;
    for (int itr = 0; itr < ITERATIONS ; itr ++)
    {
        nextError = IterativeRefinement(imgsToCalib,
                            newObjectPoints,
                            imagePoints,
                            cameraMatrix ,
                            distCoeffs,
                            rvecs,
                            tvecs,
                            BoardSize,
                            avr ,
                            itr);  

        if (nextError > prevError)
        {
            break;
        }
        else
        {
            prevError = nextError;
        }
        
    }
     
    return res;
}