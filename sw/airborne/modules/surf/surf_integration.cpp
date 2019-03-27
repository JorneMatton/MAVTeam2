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

const float DIST_TRESHOLD = 0.25;             // euclidean distance threshold for the SURF descriptor match filter
const int N_SKIP = 2;                        // gap in images between frameNum matching reset
const float TEMP_SIZE_FACTOR = 1.5;          // ratio factor (template area / feature_size) for template matching
const float OBJECT_SCALE_DETECTION_TH = 1.2; // scale increase detection treshold
const float ERROR_DECREASE_FACTOR = 0.6;     // factor by which the template matching error must have improved compared to scale 1
const int TEMP_MATCH_NUM_OF_SCALE_IT = 10;   // number of iterations in the scaling template matching procedure
const double SURF_HESSIAN_TRESHOLD = 50;   // threshold of hessian for the SURF detection -> depends on frameNum quality
const bool SURF_IS_UPRIGHT = false;          // Use U-surf to disregard rotation invariance for performance boost
const bool SURF_IS_EXTENDED = false;         // set the surf from 64 dimensions to 128 (slower matching)
const bool EDGE_DETECTOR_IS_ON = false;     // Apply edge detector at template matching
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

    //Define region of interest
    Mat mask = Mat::zeros(newImg.size(), CV_8U);  // type of mask is CV_8U
    cv::Rect region(imgWidth/2-240/2,imgHeigth/2-240/2, 240,240);
    Mat roi(mask,region);
    roi = Scalar(255); 

    //Obtain surf features
    Ptr<SURF> detector = SURF::create(SURF_HESSIAN_TRESHOLD, 4, 3, SURF_IS_EXTENDED, SURF_IS_UPRIGHT);
    vector<KeyPoint> newKps;
    Mat newDescs;
    detector->detectAndCompute(grayNewImg, mask, newKps, newDescs);

    //we will store object keypoints in here
    vector<float> objectYPoints;

    // Skip the first N_SKIP images since we cannot match them with anything yet
    if (prevImgQueue.size() > (N_SKIP - 1))
    {
        //Unpack attributes of the oldest image in the queue to match with
        Mat prevImg = prevImgQueue.front();
        prevImgQueue.pop_front();
        Mat prevDescs = prevDescsQueue.front();
        prevDescsQueue.pop_front();
        vector<KeyPoint> prevKps = prevKpsQueue.front();
        prevKpsQueue.pop_front();

        //Match with the previous image and its descriptors and keypoints
        BFMatcher bf = BFMatcher(NORM_L2, true);
        vector<DMatch> initialMatches, finalizedMatches;
        if (prevDescs.empty() != true && newDescs.empty() != true)
        {
            bf.match(prevDescs, newDescs, initialMatches);
        }

        vector<KeyPoint> finalNewKps, finalPrevKps;

        //Filter matches above the euclidean distance treshold and keep them only if the size of the keypoint became bigger
        for (size_t idx = 0; idx < initialMatches.size(); idx++)
        {
            int prevKptIdx = initialMatches[idx].queryIdx;
            int newKptIdx = initialMatches[idx].trainIdx;
            float sizePrevKpt = prevKps[prevKptIdx].size;
            float sizeNewKpt = newKps[newKptIdx].size;

            if ((initialMatches[idx].distance) < DIST_TRESHOLD && (sizeNewKpt > sizePrevKpt))
            {

                finalizedMatches.push_back(initialMatches[idx]);
                finalNewKps.push_back(newKps[initialMatches[idx].trainIdx]);
                finalPrevKps.push_back(prevKps[initialMatches[idx].queryIdx]);
            }
        }

        //Confirm scale by template matching
        //First create template image from the previous image
        for (size_t idx = 0; idx < finalizedMatches.size(); idx++)
        {
            //Define the template size
            int templateLength = round(finalPrevKps[idx].size * TEMP_SIZE_FACTOR);

            //Check if the desired template of the previous keypoint would exceed the image size
            if ((int)(finalPrevKps[idx].pt.x + templateLength / 2 > imgWidth) || (int)(finalPrevKps[idx].pt.x - templateLength / 2 < 0) || (int)(finalPrevKps[idx].pt.y + templateLength / 2 > imgHeigth) || (int)(finalPrevKps[idx].pt.y - templateLength / 2 < 0))
            {
                continue;
            }
            else
            {
                //Create template around previous keypoint
                Size patchSize = Size(templateLength, templateLength);
                Mat prevTemplate;
                getRectSubPix(prevImg, patchSize, finalPrevKps[idx].pt, prevTemplate);

                if (EDGE_DETECTOR_IS_ON == true) // Turn on edge detector for more robustness
                {
                    Canny(prevTemplate, prevTemplate, 50, 200);
                }

                // Loop through the template method by scaling
                float lowestError = INFINITY;
                float errAtScaleOne = INFINITY;
                float bestMatchingScale = 1;
                vector<double> scales = linspace(1.0, 1.5, TEMP_MATCH_NUM_OF_SCALE_IT);

                for (size_t it = 0; it < scales.size(); ++it)
                {

                    // expand the previous template at scale
                    float scale = scales[it];
                    Mat resizedPrevTemp;
                    resize(prevTemplate, resizedPrevTemp, Size(), scale, scale, TEMPLATE_IP_METHOD);

                    // Create a template in the new image using the matched keypoint.
                    // First check if this template would not collide with the image dimensions
                    if ((int)(finalNewKps[idx].pt.x + templateLength / 2 > imgWidth) || (int)(finalNewKps[idx].pt.x - templateLength / 2 < 0) || (int)(finalNewKps[idx].pt.y + templateLength / 2 > imgHeigth) || (int)(finalNewKps[idx].pt.y - templateLength / 2 < 0))
                    {
                        break; //stop template matching procedure
                    }
                    else
                    {
                        //create the template around the matching keypoint
                        Mat newTemplate;
                        getRectSubPix(grayNewImg, resizedPrevTemp.size(), finalNewKps[idx].pt, newTemplate);

                        if (EDGE_DETECTOR_IS_ON == true) // Turn on edge detector for more robustness
                        {
                            Canny(newTemplate, newTemplate, 50, 200);
                        }

                        // Mat final_frame;
                        // hconcat(resizedPrevTemp, newTemplate, final_frame);
                        // cv::namedWindow("templates", WINDOW_NORMAL);
                        // resizeWindow("templates", 600, 600);
                        // imshow("templates", final_frame);
                        // waitKey(100);

                        // //Now compare the previous and new template using mean squared error.
                        // double MSE = cv::norm(resizedPrevTemp, newTemplate);
                        // float err = MSE * MSE / resizedPrevTemp.total();                        
                        double MSE = cv::norm(resizedPrevTemp, newTemplate, NORM_L1);
                        float err = MSE/ (scale*scale);

                        // if the error is lower, update the best match for the previous template
                        if (err < lowestError)
                        {
                            lowestError = err;
                            bestMatchingScale = scale;
                        }
                        // As a reference store the matching factor at scale 1 which we will use later
                        if (scale == 1)
                        {
                            errAtScaleOne = err;
                        }
                    }
                }

                // Mark the keypoints in the new image as objects if the following condition holds:
                if ((bestMatchingScale >= OBJECT_SCALE_DETECTION_TH) && (lowestError <= (ERROR_DECREASE_FACTOR * errAtScaleOne)))
                {
                    objectYPoints.push_back(finalNewKps[idx].pt.y);
                }
            }
        }
        // prepare variables for sorting
        std::vector<int> keypointsx(objectYPoints.begin(), objectYPoints.end());
        int amount_keypointsx = keypointsx.size();
        VERBOSE_PRINT("Number of keypoints %d \n", amount_keypointsx);
        // sort keypoints into 3 zones (as defined in the settings)
        sort_rows(keypointsx,amount_keypointsx,imgWidth,zone1,zone2,zone3);
    }

    //Add the new gray image, new descriptors and new keypoints to the queue
    prevImgQueue.push_back(grayNewImg);
    prevDescsQueue.push_back(newDescs);
    prevKpsQueue.push_back(newKps);
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