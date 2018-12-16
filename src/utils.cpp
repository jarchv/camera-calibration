#include "utils.h"

bool isIncluded(std::vector<cv::Point> X, cv::Point Pt)
{
    double dc;
    for(size_t ii = 0; ii < X.size(); ii ++)
    {
        dc = sqrt((X[ii].x - Pt.x)*(X[ii].x - Pt.x) + (X[ii].y - Pt.y)*(X[ii].y - Pt.y));
        if (dc <= 5.0)
            return true;
    }
    return false;
}
/*
bool centerVisited(std::vector<int> X, int Pos)
{
    for(size_t ii = 0; ii < X.size(); ii ++)
    {
        if (X[ii] != Pos)
            return ii
    }
    return false;
}*/
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
                    cv::Mat& contours_draw,
                    std::vector<std::vector<cv::Point>>& contours,
                    int& countFrame,
                    std::vector<cv::Point>& RpdCnts,
                    int& predictions)
{
    predictions = 0;
    //std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    std::vector<cv::Point> pdCnts(24);
    std::vector<cv::Point> tempCnts;
    //cv::RNG rng(12345);
    cv::Mat result;
    //cv::Mat contours_draw;
    cv::Mat contours_input;
    frame.copyTo(result);
    int cb              = 0;

    bin = cv::Mat::zeros(gray.size(), CV_8UC1);
    thresholdIntegral(gray,bin);

    bin.copyTo(contours_draw);
    cv::cvtColor(contours_draw, contours_draw, CV_GRAY2RGB);
    cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT,
                                       cv::Size( 7,7),
                                       cv::Point( -1, -1) );
    //cv::dilate( bin, threshold_output, element );
    //cv::erode(threshold_output, threshold_output, element);
    bin.copyTo(contours_input);

    findContours(contours_input, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE, cv::Point(0, 0));
    //findContours(threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE, cv::Point(0, 0));
    std::vector<cv::Point> centers(contours.size());
    std::vector<float> diag(contours.size());

    std::vector<cv::RotatedRect> minRect( contours.size() );
    //std::vector<cv::RotatedRect> minEllipse( contours.size() );
    int contours_count = 0;
    
    for( int i = 0; i < contours.size(); i++ )
    {
        if(contours[i].size() > 25  && contours[i].size() < 250)
        {
            minRect[contours_count]    = cv::minAreaRect( cv::Mat(contours[i]) );
            //minEllipse[contours_count] = cv::fitEllipse(  cv::Mat(contours[i]) );
            contours_count++;
        }
    }

    cv::Scalar color;
    cv::Point2f rect_points[4];
    int centerx;
    int centery;
    float diagtemp;
    float errormaxDiam = 5.5;
    float errormax = 5; 
    float minc;
    float dist;
    float dist_centers;
    std::vector<int> centerVisited = {0,1,2,3,4,5,6,7,8,9,10,11};

    std::vector<int> id4Center;
    std::vector<cv::Point> Center4id;

    //std::vector<3, int> noMatched;
    int center_jump = 0;
    cv::Mat drawing = cv::Mat::zeros( bin.size(), CV_8UC3 );

    color = cv::Scalar( 255, 250, 50);
    for( int i = 0; i< contours_count; i++ )
    {
        
        minRect[i].points( rect_points );
        centerx = 0.5*(rect_points[0].x + rect_points[2].x);
        centery = 0.5*(rect_points[0].y + rect_points[2].y);
        /*
        cv::line(contours_draw, cv::Point(rect_points[0].x,rect_points[0].y), 
                                cv::Point(rect_points[2].x,rect_points[2].y), 
                                cv::Scalar( 0, 255, 0), 2, 8, 0);
        */
        centers[i] = cv::Point(centerx,centery); 
        diag[i] =  sqrt((rect_points[2].x - rect_points[0].x)*(rect_points[2].x - rect_points[0].x)+(rect_points[2].y - rect_points[0].y)*(rect_points[2].y - rect_points[0].y));
    }

    int countc=0;
  
    for(int p = 0; p < contours_count; p++){
        minRect[p].points( rect_points );
        centerx = 0.5*(rect_points[0].x + rect_points[2].x);
        centery = 0.5*(rect_points[0].y + rect_points[2].y);
        diagtemp = sqrt(pow((rect_points[2].x - rect_points[0].x),2)+pow((rect_points[2].y - rect_points[0].y),2));

        for(int k = 0; k < contours_count; k++)
        {
            if(k != p){
                dist = sqrt((centerx - centers[k].x)*(centerx - centers[k].x) + (centery - centers[k].y)*(centery - centers[k].y));
                if((dist <= errormax) && (diag[k] - diagtemp)  > errormaxDiam)
                {
                    //predictions++;
                    tempCnts.push_back(cv::Point(centerx, centery));
                    break;
                }
            }
        }
    }

    for (int i = 0; i < tempCnts.size(); i++)
    {
        centerx = tempCnts[i].x;
        centery = tempCnts[i].y;
        if(countFrame>0){
            minc = 1e5;
            for(int c =  0; c < RpdCnts.size();c++){
                dist_centers = sqrt((centerx - RpdCnts[c].x)*(centerx - RpdCnts[c].x) + (centery - RpdCnts[c].y)*(centery - RpdCnts[c].y));
                if(dist_centers < minc){
                    minc = dist_centers;
                    cb = c;
                }
            }
            
            //std::cout << "current " << countc << " predic " << cb << std::endl;
            //std::cout << "temp = " <<tempCnts.size() << std::endl;
            if(minc < 10)
            {
                circle( result, cv::Point(centerx, centery),2, cvScalar(0,0,255), 2, 8); 
                putText(result,std::to_string(cb + 1),cv::Point(centerx, centery),cv::FONT_ITALIC,0.8,color,2); 
                pdCnts[cb] = cv::Point(centerx, centery);
                centerVisited.erase(std::find(centerVisited.begin(),centerVisited.end(),cb));
                predictions++;
            }

            else
            {
                //pdCnts[countc] = cv::Point(centerx, centery);
                //circle( result, cv::Point(centerx, centery), 2, cvScalar(0,0,255), 2, 8);  
                //putText(result,std::to_string(countc+1),cv::Point(centerx, centery),cv::FONT_ITALIC,0.8,color,2); 
                //std::cout<< "center jump : " << "i = "<< i << ", cb = " << cb <<  ", tempsize = " << tempCnts.size() << ", minc = " << minc <<std::endl;
                //std::cout<< "dist = " << sqrt((centerx - RpdCnts[2].x)*(centerx - RpdCnts[2].x) + (centery - RpdCnts[2].y)*(centery - RpdCnts[2].y))<<std::endl;
                /*
                if (minc < 60)
                {
                    pdCnts[i] = cv::Point(centerx, centery);
                    circle( result, cv::Point(centerx, centery), 2, cvScalar(0,0,255), 2, 8);
                    putText(result,std::to_string(i+1),cv::Point(centerx, centery),cv::FONT_ITALIC,0.8,color,2); 

                }
                else {
                    center_jump++;
                }
                */
                //id4Center.push_back(i);
                Center4id.push_back(cv::Point(centerx,centery));
                //Center4id[]
                center_jump++;
                /*                
                                pdCnts[countc] = cv::Point(centerx, centery);
                                circle( result, cv::Point(centerx, centery), 2, cvScalar(0,0,255), 2, 8);  
                                putText(result,std::to_string(countc+1),cv::Point(centerx, centery),cv::FONT_ITALIC,0.8,color,2); 
                                newframe = true;
                */                
            }
            //countc++;
                            
        }
        else{
            pdCnts[countc] = cv::Point(centerx, centery);
            cv::circle( result, cv::Point(centerx, centery), 2, cv::Scalar(0,0,255), 2, 8);  
            countc++;
        }        
    }
    
    //std::cout<< "center_jump = " << center_jump <<std::endl;

    if (center_jump >= 4){
        for (int i = 0; i < 12; i++)
        {
            centerx = tempCnts[i].x;
            centery = tempCnts[i].y;
            predictions++;
            pdCnts[i] = cv::Point(centerx, centery);
            circle( result, cv::Point(centerx, centery), 2, cvScalar(0,0,255), 2, 8);
            putText(result,std::to_string(i+1),cv::Point(centerx, centery),cv::FONT_ITALIC,0.8,color,2); 
        }
      
    }
    
    else if (center_jump > 0)
    {
        if (centerVisited.size() > 0)
        {

            int rr = std::min(centerVisited.size(), Center4id.size());
            for (int i = 0; i < rr; i++)
            {
                int newid = centerVisited[i];
                centerx = Center4id[i].x;
                centery = Center4id[i].y;
                predictions++;
                circle( result, cv::Point(centerx, centery), 2, cvScalar(0,0,255), 2, 8);
                putText(result,std::to_string(newid+1),cv::Point(centerx, centery),cv::FONT_ITALIC,0.8,color,2);  
                pdCnts[newid] = cv::Point(centerx, centery);           
            }
        }

    }
    //predictions = pdCnts.size();
    //std::cout << "pd :" << predictions << std::endl;
    RpdCnts = pdCnts;
    countFrame++;

    return result;
}