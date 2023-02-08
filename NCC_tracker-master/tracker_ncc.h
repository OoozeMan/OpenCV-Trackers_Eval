#ifndef TRACKER_NCC_H
#define TRACKER_NCC_H


#include <cmath>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <vector>
#include <time.h>
#include "cimage.h"
#include "tracker_mosse.h"

using namespace cv;
using namespace std;

//Struct that contains image frame and its relatives
//struct image_track {
//    Mat real_image;
//    Mat image_spectrum;
//    Mat filter_output;
//
//    int cols;
//    int rows;
//
//    int opti_dft_comp_rows;
//    int opti_dft_comp_cols;
//};

class Tracker_NCC
{
public:
    Tracker_NCC();
    ~Tracker_NCC();

    image_track prev_img;                               //Previous frame
    image_track current_img;                            //Current frame

    ROI_track prev_ROI;                                 //Object location within previous frame
    ROI_track current_ROI;                              //Object location within current frame

    float NCC_tracker(Mat, Mat, Rect, bool, bool*, Rect*);            //Main NCC Tracker
    void InitTracker_NCC(Mat, Rect);                    //Initialize NCC tracker

    bool occl_flag;
    Mat blended_template;
    int NCC_occl_cnt;

};

#endif // TRACKER_H