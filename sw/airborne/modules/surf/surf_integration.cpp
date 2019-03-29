#include <stdio.h>
#include "std.h"
#include <iostream>
#include "opencv2/core.hpp"
#include "opencv2/features2d.hpp"
#include "opencv2/xfeatures2d.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/xfeatures2d/nonfree.hpp"
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <deque>
#include <numeric>

#include "surf_integration.h"


#define PRINT(string,...) fprintf(stderr, "[surf_integration->%s()] " string,__FUNCTION__ , ##__VA_ARGS__)
#define VERBOSE_PRINT PRINT

const float DIST_TRESHOLD = 0.2;             // euclidean distance threshold for the SURF descriptor match filter
const int N_SKIP = 2;                        // gap in images between frameNum matching reset
const float TEMP_SIZE_FACTOR = 4;          // ratio factor (template area / feature_size) for template matching
const float OBJECT_SCALE_DETECTION_TH = 1.2; // scale increase detection treshold
const float ERROR_DECREASE_FACTOR = 0.8;     // factor by which the template matching error must have improved compared to scale 1
const int TEMP_MATCH_NUM_OF_SCALE_IT = 15;   // number of iterations in the scaling template matching procedure
const double SURF_HESSIAN_TRESHOLD = 150;   // threshold of hessian for the SURF detection -> depends on frameNum quality
const bool SURF_IS_UPRIGHT = true;          // Use U-surf to disregard rotation invariance for performance boost
const bool SURF_IS_EXTENDED = false;         // set the surf from 64 dimensions to 128 (slower matching)
const bool EDGE_DETECTOR_IS_ON = false;      // Apply edge detector at template matching
const int DECLARE_AS_OBSTACLE_TH = 3; //mininum number of keypoints that have to be detected

const float OUTER_ZONES_FRAC = 0.5; //fraction of the images the outer zones take in
const float MIDDLE_ZONE_FRAC = 0.2; //fraction of the images the inner zone takes in

#define TEMPLATE_IP_METHOD INTER_LINEAR      // template resizing method

using namespace std;
using namespace cv;
using namespace cv::xfeatures2d;

deque<Mat> prevImgQueue, prevDescsQueue;
deque<vector<KeyPoint>> prevKpsQueue;

std::vector<double> linspace(double start, double end, int num);
void sort_rows(vector<int> keypointsx, int amount_keypoints, int imgWidth, int *zone1, int *zone2, int *zone3);


//Main
void surfDetectObjectsAndComputeControl(char *img, int imgWidth, int imgHeigth, int *zone1, int *zone2, int *zone3)
{   
    
    //Get the image
    Mat newImg(imgHeigth, imgWidth, CV_8UC2, img);

    //Convert to grayscale
    Mat grayNewImg;
    cvtColor(newImg, grayNewImg, CV_YUV2GRAY_Y422);

    int ROIwidth = 50;
    int ROIheigth = 50;
        
    //Define region of interest
    Mat mask = Mat::zeros(newImg.size(), CV_8U);  // type of mask is CV_8U
    cv::Rect region(imgWidth/2-ROIwidth/2,imgHeigth/2-ROIheigth/2, ROIwidth,ROIheigth);
    Mat roi(mask,region);
    roi = Scalar(255); 

    //Obtain surf features
    Ptr<SURF> detector = SURF::create(SURF_HESSIAN_TRESHOLD, 4, 3, SURF_IS_EXTENDED, SURF_IS_UPRIGHT);
    vector<KeyPoint> newKps;
    Mat newDescs;
    detector->detectAndCompute(grayNewImg, mask, newKps, newDescs);

    //we will store object keypoints in here
    vector<float> objectYPoints;
    for(uint idx=0;idx<newKps.size();idx++){
        objectYPoints.push_back(newKps[idx].pt.y);
    }
    
    

    // prepare variables for sorting
    std::vector<int> keypointsx(objectYPoints.begin(), objectYPoints.end());
    int amount_keypointsx = keypointsx.size();
    VERBOSE_PRINT("Number of keypoints %d \n", amount_keypointsx);

    // sort keypoints into 3 zones (as defined in the settings)
    sort_rows(keypointsx,amount_keypointsx,imgWidth,zone1,zone2,zone3);    

   
}

std::vector<double> linspace(double start, double end, int num)
{
    std::vector<double> linspaced;

    if (num == 0)
    {
        return linspaced;
    }
    if (num == 1)
    {
        linspaced.push_back(start);
        return linspaced;
    }

    double delta = (end - start) / (num - 1);

    for (int i = 0; i < num - 1; ++i)
    {
        linspaced.push_back(start + delta * i);
    }
    linspaced.push_back(end); // I want to ensure that start and end
                              // are exactly the same as the input

    return linspaced;
}


void sort_rows(vector<int> keypointsx, int amount_keypoints, int imgWidth, int *zone1, int *zone2, int *zone3)
{
        // initialise outer and inner zone
        int outer_zones_width = (int)(OUTER_ZONES_FRAC * imgWidth);
        int inner_zone_width = (int)(MIDDLE_ZONE_FRAC * imgWidth);

        int border1 = 0;
        int border2 = (int)(0.5 * imgWidth - inner_zone_width / 2);
        int border3 = outer_zones_width;
        int border4  = imgWidth-outer_zones_width;
        int border5 = (int)(0.5 * imgWidth + inner_zone_width / 2);
        int border6 = imgWidth;

        VERBOSE_PRINT("borders, %d,%d,%d,%d,%d,%d \n",border1,border2,border3,border4,border5,border6);

        for(int i = 0; i < amount_keypoints; i++)
        {
            if (border1 <= keypointsx[i] && keypointsx[i] <= border3){
                *zone1 += 1;
            }
            if (border2 <= keypointsx[i] && keypointsx[i] <= border5){
                *zone2 += 1;
            }
            if (border4 <= keypointsx[i] && keypointsx[i] <= border6){
                *zone3 += 1;
            }
        }
        return;
}