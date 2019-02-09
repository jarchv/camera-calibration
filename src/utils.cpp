#include "utils.h"
#include <algorithm>

#include <opencv2/calib3d/calib3d.hpp>

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


void IterativeRefinement(std::vector<cv::Mat> imgsToCalib,
                         std::vector<cv::Point3f> objectPoints,
                         std::vector<std::vector<cv::Point2f>> imagePoints,
                         cv::Mat& cameraMatrix , cv::Mat& distCoeffs,
                         std::vector<cv::Mat>& rvecs, std::vector<cv::Mat>& tvecs,
                         cv::Size BoardSize,
                         double& getAvr)
{
    cv::Mat dst;
    float BIAS = 200;

    std::vector<std::vector<cv::Point3f>> NewObjectPointsModel(imagePoints.size());
    std::vector<std::vector<cv::Point2f>> imagePointsReProyected(imagePoints.size());
    /*
    

    std::vector<std::vector<cv::Point2f>> ObjectPoints2D;
    std::vector<std::vector<cv::Point2f>> SORTimagetPoints;
    std::vector<cv::Point2f> objectP;
    std::vector<cv::Point2f> SORTP;

    std::vector<std::vector<cv::Point2f>> imagePointsReProyected(imagePoints.size());
    

    int F = BoardSize.width * BoardSize.height - 1;
    
    
    for(int i = 0; i < imagePoints.size(); i++)
    {
        objectP.clear();
        SORTP.clear();

        int pos1 = 0;
        int pos2 = BoardSize.width * BoardSize.height - 1;
        int pos3 = BoardSize.width - 1;
        int pos4 = BoardSize.width * (BoardSize.height - 1);
        
        objectP.push_back(cv::Point2f(objectPoints[pos1].x + BIAS, objectPoints[pos1].y + BIAS)     );
        objectP.push_back(cv::Point2f(objectPoints[pos2].x + BIAS, objectPoints[pos2].y + BIAS)                   );
        objectP.push_back(cv::Point2f(objectPoints[pos3].x + BIAS, objectPoints[pos3].y + BIAS) );
        objectP.push_back(cv::Point2f(objectPoints[pos4].x + BIAS, objectPoints[pos4].y + BIAS)                   );

        SORTP.push_back(imagePoints[i][pos1]);
        SORTP.push_back(imagePoints[i][pos2]);
        SORTP.push_back(imagePoints[i][pos3]);
        SORTP.push_back(imagePoints[i][pos4]);
        
        ObjectPoints2D.push_back(objectP);
        SORTimagetPoints.push_back(SORTP);
    }


    assert (SORTimagetPoints.size() == ObjectPoints2D.size());
    */
    cv::Mat temp;
    int D = 45;
    for(int i = 0; i < imagePoints.size(); i++)
    {
        temp = imgsToCalib[i].clone();
  
        std::vector<cv::Point2f> ObjectPointsModel2D;
        for(int j = 0; j < objectPoints.size(); j++)
        {
            ObjectPointsModel2D.push_back(cv::Point2f(objectPoints[j].x + BIAS, D * (BoardSize.height - 1) -  objectPoints[j].y + BIAS/2));
        }

        std::vector<cv::Point2f> imagePointsModel2D;

        for(int j = 0; j < imagePoints[i].size(); j++)
        {
            imagePointsModel2D.push_back(cv::Point2f(imagePoints[i][j].x, imagePoints[i][j].y));
        }
        //std::cout << ObjectPointsModel2D << std::endl;
        //std::cout << imagePointsModel2D  << std::endl;
        //cv::Mat M = cv::getPerspectiveTransform(SORTimagetPoints[i], ObjectPoints2D[i]);
        cv::Mat M = cv::findHomography(imagePointsModel2D, ObjectPointsModel2D);

        if (M.cols != 3 || M.rows != 3)
        {
            //std::cout << "M + " << std::endl;
            //NewObjectPointsModel.push_back(objectPoints);
            NewObjectPointsModel[i] = objectPoints;
            //std::cout << NewObjectPointsModel[i] << std::endl; 
            cv::projectPoints(NewObjectPointsModel[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePointsReProyected[i]);
            continue;
        }
        cv::undistort(temp, imgsToCalib[i], cameraMatrix, distCoeffs);
        warpPerspective(imgsToCalib[i], dst, M, imgsToCalib[i].size());
        //cv::flip(dst,dst,0);
        
        

        std::vector<cv::Point> SortedPoints2;
        cv::Mat gray2;
        cv::Mat bin2;
        cv::Mat contours_draw2;
        std::vector<std::vector<cv::Point>> contours2;
        
        int c = 0;

        cv::cvtColor(dst, gray2, cv::COLOR_BGR2GRAY);
        cv::GaussianBlur(gray2, gray2, cv::Size(5,5), 0, 0);  

        cv::Mat result2 = findCenters(dst, gray2, bin2, contours_draw2, contours2, c, SortedPoints2, BoardSize, 0.05);

        //cv::Rect myROI(200 - D * 1.3, D*2.5 , D * (BoardSize.width + 1.5), D * (BoardSize.height + 0.5));
        cv::Rect myROI(BIAS - D * 1.0, D*1.5 , D * (BoardSize.width + 1.2), D * (BoardSize.height + 0.5));
        cv::Mat croppedRef(result2, myROI);


        if (SortedPoints2.size() == (BoardSize.width * BoardSize.height))
        {
            drawLines(result2, SortedPoints2);
            std::vector<cv::Point3f> FimgPoint; 
            for(int j = 0; j < SortedPoints2.size(); j++)
            {
                FimgPoint.push_back(cv::Point3f(SortedPoints2[j].x - BIAS , D * (BoardSize.height - 1) - (SortedPoints2[j].y - BIAS/2), 0.0));
            }
            //NewObjectPointsModel.push_back(FimgPoint);
            NewObjectPointsModel[i] = FimgPoint;
        }
        else{
            //NewObjectPointsModel.push_back(objectPoints);
            NewObjectPointsModel[i] = objectPoints;
        }

        //std::cout << NewObjectPointsModel[i] << std::endl; 
        //cv::imshow("front-to-parallel",dst);
        //cv::imshow("result2", result2);
        cv::imshow("crop", croppedRef);        
        cv::waitKey(30);    
        cv::projectPoints(NewObjectPointsModel[i], rvecs[i], tvecs[i], cameraMatrix, distCoeffs, imagePointsReProyected[i]);
        /*
        std::cout << "======>" << std::endl;
        std::cout << imagePointsReProyected[i] << std::endl; 
        std::cout << "======" << std::endl;
        std::cout << imagePoints[i] << std::endl; 
        std::cout << "<======" << std::endl;
        */
    }
    cv::destroyAllWindows();

    for (int i = 0; i < imagePointsReProyected.size(); i++)
    {
        //std::cout << NewObjectPointsModel[i] << std::endl;
        for (int j = 0; j < imagePointsReProyected[i].size(); j++)
        {
            imagePointsReProyected[i][j].x = (imagePoints[i][j].x)*0.5 + (imagePointsReProyected[i][j].x)*0.5;
            imagePointsReProyected[i][j].y = (imagePoints[i][j].y)*0.5 + (imagePointsReProyected[i][j].y)*0.5;
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

    cameraMatrix.at<double>(0,0) = 4.0/3.0;
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

    
    for (int i = 0; i < 20 ; i ++)
    {
        IterativeRefinement(imgsToCalib,
                            newObjectPoints,
                            imagePoints,
                            cameraMatrix ,
                            distCoeffs,
                            rvecs,
                            tvecs,
                            BoardSize,
                            avr );  
    }
     
    return res;
}