#include "utils.h"

bool isIncluded(std::vector<cv::Point> X, cv::Point Pt)
{
    for(size_t ii = 0; ii < X.size(); ii ++)
    {
        double dc = sqrt((X[ii].x - Pt.x)*(X[ii].x - Pt.x) + (X[ii].y - Pt.y)*(X[ii].y - Pt.y));
        if (dc <= 4)
            return true;
    }
    return false;
}

void thresholdIntegral(cv::Mat &inputMat, cv::Mat &outputMat)
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

    int S = MAX(nRows, nCols)/8;
    double T = 0.15;

    // perform thresholding
    int s2 = S/2;
    int x1, y1, x2, y2, count, sum;

    // CV_Assert(sizeof(int) == 4);
    int *p_y1, *p_y2;
    uchar *p_inputMat, *p_outputMat;
//#pragma parallel for
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

cv::Mat findCenters(cv::Mat frame, 
                    cv::Mat gray, 
                    cv::Mat& bin, 
                    cv::Mat& threshold_output,
                    int& countFrame,
                    std::vector<cv::Point>& RpdCnts)
{

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    std::vector<cv::Point> pdCnts(13);

    cv::Mat result;

    frame.copyTo(result);
    int cb              = 0;

    bin = cv::Mat::zeros(gray.size(), CV_8UC1);
    thresholdIntegral(gray,bin);

    //cv::moveWindow("threshold",1040,40);
    //cv::imshow("threshold", bin);

    cv::Mat element = cv::getStructuringElement( cv::MORPH_ELLIPSE,
                                       cv::Size( 7,7),
                                       cv::Point( 0, 0 ) );
    cv::dilate( bin, threshold_output, element );
    //erode(bin, threshold_output, element);

    findContours(threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE, cv::Point(0, 0));

    std::vector<cv::Point> centers(contours.size());
    std::vector<float> radius(contours.size());

    std::vector<cv::RotatedRect> minRect( contours.size() );
    std::vector<cv::RotatedRect> minEllipse( contours.size() );
    int counte = 0;
    
    for( int i = 0; i < contours.size(); i++ )
    {
        if( contours[i].size() > 5  && contours[i].size() < 250)
        {
            minRect[counte]    = cv::minAreaRect( cv::Mat(contours[i]) );
            minEllipse[counte] = cv::fitEllipse(  cv::Mat(contours[i]) );
            counte++;
        }
    }

    cv::Scalar color;
    cv::Point2f rect_points[4];
    int centerx;
    int centery;
    float radiustemp;
    float errormaxDiam = 5.5;
    float errormax = 2; 
    float minc;
    cv::Mat drawing = cv::Mat::zeros( threshold_output.size(), CV_8UC3 );

    for( int i = 0; i< counte; i++ )
    {
        color = cv::Scalar( 255, 255, 0);
        minRect[i].points( rect_points );
        centerx = 0.5*(rect_points[0].x + rect_points[2].x);
        centery = 0.5*(rect_points[0].y + rect_points[2].y);
        centers[i] = cv::Point(centerx,centery); 
        radius[i] =  sqrt((rect_points[2].x - rect_points[0].x)*(rect_points[2].x - rect_points[0].x)+(rect_points[2].y - rect_points[0].y)*(rect_points[2].y - rect_points[0].y));
    }
    int countc=0;
  
    for(int p = 0;p<counte;p++){
        minRect[p].points( rect_points );
        centerx = 0.5*(rect_points[0].x + rect_points[2].x);
        centery = 0.5*(rect_points[0].y + rect_points[2].y);
        int count = 0;
        radiustemp = sqrt(pow((rect_points[2].x - rect_points[0].x),2)+pow((rect_points[2].y - rect_points[0].y),2));
        for(int k = 0;k<counte;k++)
        {
            if(k != p){
                float dist = sqrt((centerx - centers[k].x)*(centerx - centers[k].x) + (centery - centers[k].y)*(centery - centers[k].y));
                if((dist <= errormax) && (radius[k] - radiustemp)  > errormaxDiam)
                {
                    count++;
                }
                if(count > 0){
                    if (isIncluded(pdCnts, cv::Point(centerx, centery)) == false)
                    {
                        if(countFrame>0){
                            minc = 1e5;
                            for(int c =  0;c<RpdCnts.size();c++){
                                float dist_centers = sqrt((centerx - RpdCnts[c].x)*(centerx - RpdCnts[c].x) + (centery - RpdCnts[c].y)*(centery - RpdCnts[c].y));
                                if(dist_centers<minc){
                                    minc = dist_centers;
                                    cb = c;
                                }
                            }
                            
                            //std::cout << "current " << countc << " predic " << cb << std::endl;
                            if(minc < 9)
                            {
                                circle( result, cv::Point(centerx, centery),2, cvScalar(0,0,255), 2, 8); 
                                putText(result,std::to_string(cb),cv::Point(centerx, centery),cv::FONT_ITALIC,1.0,color,2); 
                                pdCnts[cb] = cv::Point(centerx, centery);
                            }

                            else 
                            {
                                pdCnts[countc] = cv::Point(centerx, centery);
                                circle( result, cv::Point(centerx, centery), 2, cvScalar(0,0,255), 2, 8);  
                                putText(result,std::to_string(countc),cv::Point(centerx, centery),cv::FONT_ITALIC,1.0,color,2); 
                            }
                            countc++;
                        }
                        else{
                            pdCnts[countc] = cv::Point(centerx, centery);
                            cv::circle( result, cv::Point(centerx, centery), 2, cv::Scalar(0,0,255), 2, 8);  
                            countc++;
                        }
                    } 
                    break;
                } 
            }
        }
    }
    RpdCnts = pdCnts;

 

    countFrame++;

    return result;
}