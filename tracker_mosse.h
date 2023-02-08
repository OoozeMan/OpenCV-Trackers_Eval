/**********************************************************************/
/***                                                                ***/
/***    Adaptive Correlation Filters Tracking Implementation        ***/
/***                                                                ***/
/**********************************************************************/
/**********************************************************************/
/***                                                                ***/
/***    Based on :                                                  ***/
/***                                                                ***/
/***    Visual Object Tracking using Adaptive Correlation Filters   ***/
/***                                                                ***/
/***    - David S. Bolme - J. Ross Beveridge -                      ***/
/***    - Bruce A. Draper - Yui Man Lui -                           ***/
/***    Computer Science Dpt, Colorado State University             ***/
/***    bolme@cs.colostate.edu                                      ***/
/***                                                                ***/
/**********************************************************************/
/**********************************************************************/
/***                                                                ***/
/***    Implementation by :                                         ***/
/***                                                                ***/
/***    - Alberto Quintero Delgado                                  ***/
/***      ajquinterod@gmail.com                                     ***/
/***                                                                ***/
/***    - Deray Jeremie                                             ***/
/***      deray.jeremie@gmail.com                                   ***/
/***                                                                ***/
/***    Master in Computer Vision                                   ***/
/***    University of Burgundy                                      ***/
/***    January 2014                                                ***/
/***                                                                ***/
/**********************************************************************/

#ifndef TRACKER_MOSSE_H
#define TRACKER_MOSSE_H

#define CVPLOT_HEADER_ONLY

#include <cmath>
#include <opencv2/opencv.hpp>
#include <stdio.h>
#include <vector>
#include <time.h>
#include "cimage.h"
//#include "CvPlot/cvplot.h"

#define OCCLUDED 0
#define LOST 1
#define FOUND 2

using namespace cv;
using namespace std;

//Struct that contains image frame and its relatives
struct image_track{
    Mat real_image;
    Mat image_spectrum;
    Mat filter_output;

    int cols;
    int rows;

    int opti_dft_comp_rows;
    int opti_dft_comp_cols;
};

//Struct ROI, tracked object
struct ROI_track{
    Rect ROI;
    Point ROI_center;
};

//Struct maintaining a history of last 50 frames
struct Frame_History
{
    int frameID;
    Mat frame;
    Rect roi;
    double score;

};

//Tracker class
class Tracker_MOSSE //: public QObject //Inherit from QObject class
{

  //  Q_OBJECT

public:
    Tracker_MOSSE();
    ~Tracker_MOSSE();

    image_track prev_img;                               //Previous frame
    image_track current_img;                            //Current frame

    ROI_track prev_ROI;                                 //Object location within previous frame
    ROI_track current_ROI;                              //Object location within current frame

    //void InitTracker(const Mat & , QPoint, QPoint);     //Init tracker from user selection
    void InitTracker(const Mat & , Rect, Rect);

    void Track(const Mat&);                             //Perform tracking over current frame

    bool isInitialized() const;                         //Verify tracker is init

    Rect getPosition() const;                           //Get ROI position

    int getState() const;                               //Get the state { FOUND, OCCLUDED, LOST }

    Mat GetFilter() const;                              //Get filter

    void SetPSR_mask(int);                              //Set PSR var
    void SetPSR_ratio_low(int);
    void SetPSR_ratio_high(int);

    int GetPSR_mask() const;                            //Get PSR var
    int GetPSR_ratio_low() const;
    int GetPSR_ratio_high() const;

    float Get_Learning() const;                         //Get/Set learning ratio
    void Set_Learning(float);

    float calculate_ssd (Mat, Mat);                    // Calculate Sum-of-Square-Difference SSD

    float MOSSE_TRACKER (Mat, Point, Mat, int, int, bool, Rect*, int*);

    float PSR_value;                                    // Update the PSR value
    float PSR_value_old;                                // PSR value for filter not being updated
    int Occlusion_cnt;                                  //Counter number of successive Occlusions
    float Canny_score_orig;                             //Score of Canny Edge at TrackerInit
    int tracking_cnt;                                   // Count number of successful trackings

    bool occlusion_flag_ssd = false;

    int state_;
    bool ssd_flag;
    bool occlusion_flag;
    double match_templateVal;

    Mat blended_template;

    Frame_History Tracker_History (Mat, Rect, int, float, int); // Track History of n-number of frames

    void InitHist_Struct(Mat, Rect, int); //Initialize History Struct
 
    void Annotate_Tracker(Mat, Rect, int, float); // Annotate Tracker results onto frame

    float Occlusion_Test(Mat, Mat); // Test Occlusion by dividing templates in upper/lower and left/right halves

    void Hassan_Grid_Search(Mat, Rect, Mat, float*, int*, int*); // Refine search of Tracker results


//signals:
//
//    void newCorrelationImage(const QImage &img);        //Qt signals
//    void newFilterImage(const QImage &img);
//    void newInputImage(const QImage &img);

private:

    Mat _filter;                                        //Tracker filter
    Mat _filterOld;                                     //Tracker filter not updated
    Mat _HanningWin;                                    //Pre-processing Hanning-window
    bool _eps;
    bool _init;
    int PSR_mask;                                       //PSR var
    double PSR_ratio[2];
    double _learning;                                   //Learning ratio
    Size _im_size;                                      //Full frame size
    int frame_id;

    void InitFilter(const Mat&);                        //Init filter from user selection
    void InitFilter();

    //Apply same randomly defined affine transform to both input matrice
    void AffineTransform(const Mat&,const Mat&, Mat&, Mat&);

    //Compute Direct Fourier Transform on input image, with or without a pre-processing
    void ComputeDFT(image_track&, bool preprocess = false);
    Mat ComputeDFT(const Mat&, bool preprocess = false);

    //Init ROI position and center
    void SetRoi(Rect);

    //Update ROI position
    void UpdateRoi(Point,bool);

    //Perform tracking over current frame
    Point PerformTrack();

    //Compute Peak-to-Sidelobe Ratio
    float ComputePSR(const Mat &);

    //Update Tracker
    void Update(Point);

    //Update filter
    void UpdateFilter();

    //Create 2D Gaussian
    void MaskDesiredG(cv::Mat &,int u_x,int u_y,double sigma = 2, bool norm_energy = true);

    //Inverse DFT and save image
    void inverseAndSave(const Mat &, const std::string &, const bool &shift = false);

    //Compute complex conjugate
    Mat conj(const Mat&);

    //Compute complex divison
    void dftDiv(const Mat&, const Mat&, Mat&);

    //Compute regularization parameter
    Mat createEps(const Mat&, double std = 0.00001);

};

#endif // TRACKER_H
