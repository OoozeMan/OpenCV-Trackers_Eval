#include "tracker_ncc.h"

Tracker_NCC::Tracker_NCC()
{
    this->occl_flag = false;
    this->NCC_occl_cnt = 0;
}

Tracker_NCC::~Tracker_NCC()
{

}

void Tracker_NCC::InitTracker_NCC(Mat frame_in, Rect roi)
{
    this->blended_template = frame_in(roi).clone();
    this->prev_ROI.ROI = roi;
    this->current_ROI.ROI = this->prev_ROI.ROI;
}

float Tracker_NCC::NCC_tracker(Mat frame_in, Mat template_frame, Rect roi, bool detection_flag, bool* NCC_occl_flag, Rect* tracker_result)
{
    Mat Template_match;
    double minVal = 0.0, maxVal = 0.0;
    Point minLoc, maxLoc, centroid;

    int eucLoc = 0;

    if (detection_flag == true)
    {
        InitTracker_NCC(frame_in, roi); // Re-Initialize the Tracker with updated Frame and RoI 
        this->blended_template = frame_in(roi).clone();
    }

    Mat frame_clone = frame_in.clone();
    matchTemplate(frame_clone, this->blended_template, Template_match, TM_CCORR_NORMED, noArray());
    minMaxLoc(Template_match, &minVal, &maxVal, &minLoc, &maxLoc, noArray());

    eucLoc = (abs(this->prev_ROI.ROI.x - maxLoc.x) + abs(this->prev_ROI.ROI.y - maxLoc.y));
    centroid = { this->prev_ROI.ROI.x + this->prev_ROI.ROI.width / 2, this->prev_ROI.ROI.y + this->prev_ROI.ROI.height / 2 };

    putText(frame_clone, "NCC Score= " + to_string(maxVal), Point(10, 70), FONT_HERSHEY_DUPLEX, 0.5, Scalar(255, 0, 0), 2);
    putText(frame_clone, "Euclidean Value= " + to_string(eucLoc), Point(10, 90), FONT_HERSHEY_DUPLEX, 0.5, Scalar(255, 0, 0), 2);


    if (maxVal > 0.97 && eucLoc < 50) //(maxVal > 0.97 && eucLoc < 50)
    {
        this->current_ROI.ROI = { maxLoc.x, maxLoc.y, this->blended_template.cols, this->blended_template.rows };
        this->blended_template = 0.95 * this->blended_template + 0.05 * frame_in(this->current_ROI.ROI);
        rectangle(frame_clone, this->current_ROI.ROI, Scalar(255, 0, 0), 2, 1, 0);
        drawMarker(frame_clone, centroid, Scalar(255, 0, 0), MARKER_DIAMOND, 50, 2, 8);
        this->occl_flag = false;
        this->NCC_occl_cnt = 0;
    }

    else
    {
        this->current_ROI.ROI = { this->prev_ROI.ROI.x, this->prev_ROI.ROI.y, this->blended_template.cols, this->blended_template.rows };
        this->blended_template = template_frame.clone(); // for slower occlusion scenarios
        //this->blended_template = this->blended_template;  // for faster occlusion scenarios
        drawMarker(frame_clone, centroid, Scalar(255, 0, 0), MARKER_STAR, 50, 2, 8);
        this->occl_flag = true;
        //cout << "NCC Occlusion" << endl;
        this->NCC_occl_cnt++;
    }

    *tracker_result = this->current_ROI.ROI;

    this->prev_ROI.ROI = this->current_ROI.ROI;
    *NCC_occl_flag = this->occl_flag;

    //imshow("NCC blended template", this->blended_template);

    return maxVal;

}