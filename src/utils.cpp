#include "utils.h"

std::vector<cv::Point> findConcentricCenters( std::vector<cv::RotatedRect> minRect,
                            std::vector<cv::Point> centers,
                            std::vector<float> diag,
                            int contours_count)
{
    std::vector<cv::Point> tempCnts;
    cv::Point2f rect_points[4];
    int centerx;
    int centery;
    float diagtemp;
    float errormaxDiam = 5.5;
    float errormax = 5; 
    float dist;
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
                    tempCnts.push_back(cv::Point(centerx, centery));
                    break;
                }
            }
        }
    }
    return tempCnts;   
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

int prevcenterx = 0;
int prevcentery = 0;
cv::Point prevCenter = cv::Point(0,0);
cv::Mat prev;
int maxdistance = 0;
cv::Mat findCenters(cv::Mat frame, 
                    cv::Mat gray, 
                    cv::Mat& bin, 
                    cv::Mat& contours_draw,
                    std::vector<std::vector<cv::Point>>& contours,
                    int& countFrame,
                    std::vector<cv::Point>& RpdCnts,
                    std::vector<cv::Point>& CurrCenters,
                    int& predictions)
{
    predictions = 0;
    std::vector<cv::Vec4i> hierarchy;
    std::vector<cv::Point> pdCnts(24);
    std::vector<cv::Point> tempCnts;
    cv::Mat result;

    cv::Mat contours_input;
    frame.copyTo(result);
    int cb              = 0;

    bin = cv::Mat::zeros(gray.size(), CV_8UC1);
    thresholdIntegral(gray,bin);

    bin.copyTo(contours_draw);
    cv::cvtColor(contours_draw, contours_draw, CV_GRAY2RGB);
    bin.copyTo(contours_input);

    findContours(contours_input, contours, hierarchy, CV_RETR_LIST, CV_CHAIN_APPROX_NONE, prevCenter);
    //findContours(threshold_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE, cv::Point(0, 0));
    std::vector<cv::Point> centers(contours.size());
    std::vector<float> diag(contours.size());

    std::vector<cv::RotatedRect> minRect( contours.size() );
    //std::vector<cv::RotatedRect> minEllipse( contours.size() );
    int contours_count = 0;
    
    for( int i = 0; i < contours.size(); i++ )
    {
        if(contours[i].size() > 25  && contours[i].size() < 280)
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

    int center_jump = 0;
    cv::Mat drawing = cv::Mat::zeros( bin.size(), CV_8UC3 );


    color = cv::Scalar( 255, 250, 50);
    for( int i = 0; i< contours_count; i++ )
    {
        
        minRect[i].points( rect_points );
        centerx = 0.5*(rect_points[0].x + rect_points[2].x);
        centery = 0.5*(rect_points[0].y + rect_points[2].y);
        centers[i] = cv::Point(centerx,centery); 
        diag[i] =  sqrt((rect_points[2].x - rect_points[0].x)*(rect_points[2].x - rect_points[0].x)+(rect_points[2].y - rect_points[0].y)*(rect_points[2].y - rect_points[0].y));
    }

    int countc=0;

    tempCnts = findConcentricCenters(minRect, centers, diag, contours_count);
    
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
                Center4id.push_back(cv::Point(centerx,centery));
                center_jump++;  
            }
                            
        }
        else{
            //centerVisited = {};
            pdCnts[countc] = cv::Point(centerx, centery);
            cv::circle( result, cv::Point(centerx, centery), 2, cv::Scalar(0,0,255), 2, 8);  
            putText(result,std::to_string(countc + 1),cv::Point(centerx, centery),cv::FONT_ITALIC,0.8,color,2); 
            //tempCnts[countc] = cv::Point(centerx, centery);
            countc++;
        }        
    }

    if (center_jump >= 4){
        
        int minx = 100000;
        int maxy = 0;
        centerVisited = {};
        for(int j = 0; j<tempCnts.size();j++){
            if(tempCnts[j].x < minx){
                minx = tempCnts[j].x;
            }
            if(tempCnts[j].y > maxy){
                maxy = tempCnts[j].y;
            }
        }
        cv::Point p1 = cv::Point(minx,maxy);

        int maxx = 0;
        int miny = 100000;

        for(int j = 0; j<tempCnts.size();j++){
            if(tempCnts[j].x > maxx){
                maxx = tempCnts[j].x;
            }
            if(tempCnts[j].y < miny){
                miny = tempCnts[j].y;
            }
        }

        cv::Point p2 = cv::Point(maxx,miny);
        int currentDistance = sqrt(pow((p2.x - p1.x),2)+pow((p2.y - p1.y),2));
        //std::cout << " max " << maxdistance << " - " << " currentd " << currentDistance << std::endl;
        if(maxdistance <= currentDistance){
            int currentcenterx = 0.5*(p1.x + p2.x);
            int currentcentery = 0.5*(p1.y + p2.y);
            //std::cout << "cx " << currentcenterx << " cy " << currentcentery << " - " << "px " << prevcenterx << " py " << prevcentery << std::endl;

            int x = currentcenterx - prevcenterx;
            int y = currentcentery - prevcentery;

            for (int i = 0; i < 12; i++)
            {
                centerx = RpdCnts[i].x + x;
                centery = RpdCnts[i].y + y;
                //differenceFrame(prev,frame);

                predictions++;
                pdCnts[i] = cv::Point(centerx, centery);
                circle( result, cv::Point(centerx, centery), 2, cvScalar(0,0,255), 2, 8);
                putText(result,std::to_string(i+1),cv::Point(centerx, centery),cv::FONT_ITALIC,0.8,color,2); 
            }
            maxdistance = currentDistance;
        }
        else{
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

        //prevCenter = cv::Point(0+x,0+y);
      
    } 
    else if (center_jump > 0)
    {
        if (centerVisited.size() > 0)
        {
            int rr = std::min(centerVisited.size(), Center4id.size());

            std::cout << centerVisited.size() << ", " << Center4id.size() << ", " << rr << std::endl;
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
        //centerVisited = {};
    }
    else{
        if(maxdistance < sqrt(pow((pdCnts[11].x - pdCnts[0].x),2)+pow((pdCnts[11].y - pdCnts[0].y),2))){
            maxdistance = sqrt(pow((pdCnts[11].x - pdCnts[0].x),2)+pow((pdCnts[11].y - pdCnts[0].y),2));
        }
    }
    prevcenterx = 0.5*(pdCnts[0].x + pdCnts[11].x);
    prevcentery = 0.5*(pdCnts[0].y + pdCnts[11].y);
    
    /*
    
    std::cout << "centerVisited.size() = " << centerVisited.size() << std::endl;
    for (int i = 0; i < centerVisited.size(); i++)
    {
        pdCnts.erase(pdCnts.begin() + i);
    }
    */
    for(int i = 0;i< 11;i++){
        cv::line(result,pdCnts[i],pdCnts[i+1],cv::Scalar(0,250,50),2);
    }
    RpdCnts = pdCnts;
    prev = frame;
    countFrame++;

    return result;
}