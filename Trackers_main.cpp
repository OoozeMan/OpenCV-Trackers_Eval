// Trackers_AT.cpp : This file contains the 'main' function. Program execution begins and ends there.
//

#include <iostream>
#include <iomanip>
#include <fstream>
#include <shobjidl.h>
#include <atlstr.h>
#include <cstring>
#include <algorithm>
#include <numeric>
#include <opencv2/core/utility.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/objdetect.hpp>
#include <conio.h>
#include <stdio.h>
#include "tracker_mosse.h"
#include "tracker_ncc.h"
#include <staple_tracker.hpp>
#include <math.h>
#include <filesystem>


using namespace std;
using namespace cv;

const float IoU_thresh = 0.0;

Rect Ground_truth(string path, int counter, int* object_state )
{
    char str[100];
    ifstream test;

    /*test.open(path + "/Resize_label" + "/Tank_Jeep_176.txt");
    Mat image = imread(path + "/Resize_frame" + "/Tank_Jeep_176.jpg");*/

    test.open(path + "v1__0" + "/labels" + "/v1_" + to_string(counter) + ".txt");
    Mat image = imread(path + "v1__0" + "/frame" + "/v1_" + to_string(counter) + ".jpg");

    Rect roi_text;
    Rect* roi_point = NULL;
    int temp, cen_x, cen_y, width, height;

    //while (test.getline(str, 40, ' '))
    //////////////// reads the first five characters with the delimiter being 'space' character
    {
        test.getline(str, 40, ' ');
        //temp = (int)(stof(str));
        *object_state = (int)(stof(str));
        test.getline(str, 40, ' ');
        cen_x = (int)(stof(str) * image.cols);
        test.getline(str, 40, ' ');
        cen_y = (int)(stof(str) * image.rows);
        test.getline(str, 40, ' ');
        width = (int)(stof(str) * image.cols);
        test.getline(str, 40, ' ');
        height = (int)(stof(str) * image.rows);

        roi_text.x = cen_x - width / 2;
        roi_text.y = cen_y - height / 2;
        roi_text.width = width;
        roi_text.height = height;

        //cout << "roi text box " << roi_text << endl;
        roi_point = &roi_text;
    }

    test.close();

    //rectangle(image, roi_text, CV_RGB(255, 255, 0), 2);
    //imshow("Ground truth ", image);

    //while (waitKey(40) == 27); //four seconds pause while the image remains displayed
    return roi_text;
}

int cnt_files(string path)
{

    int count{ };
    filesystem::path p1{ path + "v1__0" + "/frame" };
    for (auto& p : std::filesystem::directory_iterator(p1))
    {
        ++count;
    }
    std::cout << "Number of files present in " << p1 << ": " << count << '\n';

    return count;

}

void Ground_truth_annot(string path1)
{
    int count = 0;
    int count_limit = 0;
    Rect roi_text;
    int object_state;
    count_limit = cnt_files(path1);
    //while (count < count_limit)
    //{
    //    roi_text = Ground_truth(path1, count, &object_state); //function for annotating Ground truth co-ordinates obtained from text file
    //    count++;
    //}
}

void GetFileNamePath(LPWSTR pszFilePath, bool input_source) {

    HRESULT hr = CoInitializeEx(NULL, COINIT_APARTMENTTHREADED | COINIT_DISABLE_OLE1DDE);
    if (SUCCEEDED(hr))
    {
        IFileOpenDialog* pFileOpen;

        // Create the FileOpenDialog object.
        hr = CoCreateInstance(CLSID_FileOpenDialog, NULL, CLSCTX_ALL, IID_IFileOpenDialog, reinterpret_cast<void**>(&pFileOpen));

        if (SUCCEEDED(hr))
        {
            // Show the Open dialog box.
            hr = pFileOpen->Show(NULL);

            // Get the file name from the dialog box.
            if (SUCCEEDED(hr))
            {
                IShellItem* pItem;
                //IShellItem** ppsi
                if (input_source == true) // source is video
                hr = pFileOpen->GetResult(&pItem);
                else // source are image frames
                hr = pFileOpen->GetFolder(&pItem);
                //cout << "pszFilePath " << pszFilePath << endl;
                if (SUCCEEDED(hr))
                {
                    LPWSTR pTemp;
                    hr = pItem->GetDisplayName(SIGDN_FILESYSPATH, &pTemp); //SIGDN_FILESYSPATH
                    wcscpy_s(pszFilePath, MAX_PATH, pTemp);
                    if (SUCCEEDED(hr))  CoTaskMemFree(pTemp);
                    pItem->Release();
                }
            }
            pFileOpen->Release();
        }
        CoUninitialize();
    }
    return;
}

string folder_path(bool input_source)
{
    static wchar_t  szFilePath[MAX_PATH];
    GetFileNamePath(szFilePath, input_source);
    string s;
    s = CW2A(szFilePath);
    std::replace(s.begin(), s.end(), '\\', '\/'); //replace backward slashes in Folder path string with forward slashes
    if (input_source == false) // source is image frames
        s = s + '\/'; //append to end of string
    else
        s = s;
    cout << "szFilePath is " << s << endl;

    return s;
}

void plot_graph(string folder_path, int x_limit, int object_state)
{
    FILE* fp;
    int status;

    fp = _popen("gnuplot -persist", "w");
    fprintf(fp, "reset session \n");
    fprintf(fp, "set terminal wxt 1 \n");
    fprintf(fp, "set datafile separator ',' \n");
    fprintf(fp, "set ylabel '{/:Bold Jaccard Index}' \n");
    fprintf(fp, "set xlabel '{/:Bold Frame Number}' \n");
    fprintf(fp, "set xrange [0:%d] \n", (x_limit - 1));
    fprintf(fp, "set yrange [0:2] \n");
    fprintf(fp, "set title '{/:Bold Tracker Evaluation (IoU)}' \n");
    fprintf(fp, "set style fill transparent solid 0.8 noborder \n");
    fprintf(fp, "plot '%splot.csv' using 1:2 title '{/:Bold STAPLE}' with lines lw 3, ", folder_path.c_str());
    fprintf(fp, "'%splot.csv' using 1:3 title '{/:Bold MOSSE}' with lines lw 3, ", folder_path.c_str());
    fprintf(fp, "'%splot.csv' using 1:4 title '{/:Bold NCC}' with lines lw 3, ", folder_path.c_str());
    //fprintf(fp, "'%splot.csv' using 1:8:(8*($8)) title '{/:Bold Reinitialize}' with circles \n", folder_path.c_str());
    fprintf(fp, "'%splot.csv' using 1:8:9 with labels hypertext point pt 7 lc rgb '#ff000000' notitle, ", folder_path.c_str());
    fprintf(fp, "'%splot.csv' using 1:8:(2*($8)) with points pt 7 pointsize variable notitle, ", folder_path.c_str());
    fprintf(fp, "'%splot.csv' using 1:2:10 with labels hypertext point pt 7 lc rgb '#ff000000' notitle, ", folder_path.c_str());
    fprintf(fp, "'%splot.csv' using 1:3:10 with labels hypertext point pt 7 lc rgb '#ff000000' notitle, ", folder_path.c_str());
    fprintf(fp, "'%splot.csv' using 1:4:10 with labels hypertext point pt 7 lc rgb '#ff000000' notitle \n ", folder_path.c_str());
    fclose(fp);

    fp = _popen("gnuplot -persist", "w");
    //fprintf(fp, "reset session \n");
    fprintf(fp, "set terminal wxt 2 \n");
    fprintf(fp, "set datafile separator ',' \n");
    fprintf(fp, "set ylabel '{/:Bold Euclidean Distance}' \n");
    fprintf(fp, "set xlabel '{/:Bold Frame Number}' \n");
    fprintf(fp, "set xrange [0:%d] \n", (x_limit - 1));
    fprintf(fp, "set yrange [0:200] \n");
    fprintf(fp, "set title '{/:Bold Tracker Evaluation (Euclidean)}' \n");
    fprintf(fp, "set style fill transparent solid 0.8 noborder \n");
    fprintf(fp, "plot '%splot.csv' using 1:5 title '{/:Bold STAPLE}' with lines lw 3, ", folder_path.c_str());
    fprintf(fp, "'%splot.csv' using 1:6 title '{/:Bold MOSSE}' with lines lw 3, ", folder_path.c_str());
    fprintf(fp, "'%splot.csv' using 1:7 title '{/:Bold NCC}' with lines lw 3, ", folder_path.c_str());
    //fprintf(fp, "'%splot.csv' using 1:8:(8*($8)) title '{/:Bold Reinitialize}' with circles \n", folder_path.c_str());
    fprintf(fp, "'%splot.csv' using 1:8:9 with labels hypertext point pt 7 lc rgb '#ff000000' notitle, ", folder_path.c_str());
    fprintf(fp, "'%splot.csv' using 1:8:(2*($8)) with points pt 7 pointsize variable notitle, ", folder_path.c_str());
    fprintf(fp, "'%splot.csv' using 1:5:10 with labels hypertext point pt 7 lc rgb '#ff000000' notitle, ", folder_path.c_str());
    fprintf(fp, "'%splot.csv' using 1:6:10 with labels hypertext point pt 7 lc rgb '#ff000000' notitle, ", folder_path.c_str());
    fprintf(fp, "'%splot.csv' using 1:7:10 with labels hypertext point pt 7 lc rgb '#ff000000' notitle \n ", folder_path.c_str());    
    fclose(fp);

    /*
    fp = _popen("gnuplot -persist", "w");
    //fprintf(fp, "reset session \n");
    fprintf(fp, "set terminal wxt 3 \n");
    fprintf(fp, "set datafile separator ',' \n");
    fprintf(fp, "set ylabel '{/:Bold Normalized Tracker Scores}' \n");
    fprintf(fp, "set xlabel '{/:Bold Frame Number}' \n");
    fprintf(fp, "set xrange [0:%d] \n", (x_limit - 1));
    fprintf(fp, "set yrange [0:10] \n");
    fprintf(fp, "set title '{/:Bold Tracker Normalized Scores}' \n");
    fprintf(fp, "set style fill transparent solid 0.8 noborder \n");
    fprintf(fp, "plot '%splot.csv' using 1:11 title '{/:Bold STAPLE}' with lines lw 3, ", folder_path.c_str());
    fprintf(fp, "'%splot.csv' using 1:12 title '{/:Bold MOSSE}' with lines lw 3, ", folder_path.c_str());
    fprintf(fp, "'%splot.csv' using 1:13 title '{/:Bold NCC}' with lines lw 3, ", folder_path.c_str());
    //fprintf(fp, "'%splot.csv' using 1:8:(8*($8)) title '{/:Bold Reinitialize}' with circles \n", folder_path.c_str());
    fprintf(fp, "'%splot.csv' using 1:8:9 with labels hypertext point pt 7 lc rgb '#ff000000' notitle, ", folder_path.c_str());
    fprintf(fp, "'%splot.csv' using 1:8:(2*($8)) with points pt 7 pointsize variable notitle, ", folder_path.c_str());
    fprintf(fp, "'%splot.csv' using 1:11:10 with labels hypertext point pt 7 lc rgb '#ff000000' notitle, ", folder_path.c_str());
    fprintf(fp, "'%splot.csv' using 1:12:10 with labels hypertext point pt 7 lc rgb '#ff000000' notitle, ", folder_path.c_str());
    fprintf(fp, "'%splot.csv' using 1:13:10 with labels hypertext point pt 7 lc rgb '#ff000000' notitle \n ", folder_path.c_str());
    fclose(fp); 
    */
}

void plot_overlap_threshold(string folder_path, std::vector <float> staple_jacc, std::vector <float> mosse_jacc, std::vector <float> ncc_jacc, int count_limit)
{
    const int elements = 11;
    std::array<float, elements> OPE_threshold = { 0, 0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0 };
    std::array<float, elements> staple_thres = { 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0 };
    std::array<float, elements> mosse_thres = { 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0 };
    std::array<float, elements> ncc_thres = { 0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0 };

    ofstream myfile1;
    myfile1.open(folder_path + "thresh_plot.csv");
    myfile1 << "#Thresh_value, #STAPLE_Jaccard, #MOSSE_Jaccard, #NCC_Jaccard, \n";
    cout << endl;

    for (int thresh_num = 0; thresh_num < elements; thresh_num++)
    {
        for (int frame_num = 0; frame_num < count_limit - 1; frame_num++)
        {
            staple_thres[thresh_num] = (OPE_threshold[thresh_num] < staple_jacc[frame_num]) ? staple_thres[thresh_num] + staple_jacc[frame_num] : staple_thres[thresh_num];
            mosse_thres[thresh_num] = (OPE_threshold[thresh_num] < mosse_jacc[frame_num]) ? mosse_thres[thresh_num] + mosse_jacc[frame_num] : mosse_thres[thresh_num];
            ncc_thres[thresh_num] = (OPE_threshold[thresh_num] < ncc_jacc[frame_num]) ? ncc_thres[thresh_num] + ncc_jacc[frame_num] : ncc_thres[thresh_num];
        }
        //cout << "Sum of array Jaccard " << thresh_num << " " << staple_thres[thresh_num] << endl;

        myfile1 << to_string(OPE_threshold[thresh_num]) + ',';
        myfile1 << to_string(staple_thres[thresh_num] / (float)count_limit) + ',';
        myfile1 << to_string(mosse_thres[thresh_num] / (float)count_limit) + ',';
        myfile1 << to_string(ncc_thres[thresh_num] / (float)count_limit) + ',';
        myfile1 << "\n";
    }


    myfile1.close();

    FILE* fp;
    int status;
    fp = _popen("gnuplot -persist", "w");
    fprintf(fp, "reset session \n");
    fprintf(fp, "set terminal wxt 4 \n");
    fprintf(fp, "set datafile separator ',' \n");
    fprintf(fp, "set ylabel '{/:Bold Success Rate}' \n");
    fprintf(fp, "set xlabel '{/:Bold Overlap Threshold}' \n");
    fprintf(fp, "set xrange [0:%f] \n", 1.0);
    fprintf(fp, "set yrange [0:%f] \n", 1.0);
    fprintf(fp, "set xtics 0,0.1,1.0 \n", 1.0);
    fprintf(fp, "set ytics 0,0.1,1.0 \n", 1.0);
    fprintf(fp, "set title '{/:Bold Tracker Success Plots}' \n");
    fprintf(fp, "set style fill transparent solid 0.8 noborder \n");
    fprintf(fp, "plot '%sthresh_plot.csv' using 1:2 title '{/:Bold STAPLE}' with lines lw 3, ", folder_path.c_str());
    fprintf(fp, "'%sthresh_plot.csv' using 1:3 title '{/:Bold MOSSE}' with lines lw 3, ", folder_path.c_str());
    fprintf(fp, "'%sthresh_plot.csv' using 1:4 title '{/:Bold NCC}' with lines lw 3, \n", folder_path.c_str());
    fclose(fp);



}

void print_tracker_perf(std::vector <float> staple_jacc, std::vector <float> mosse_jacc, std::vector <float> ncc_jacc, std::vector <int> staple_arr_fail, std::vector <int> mosse_arr_fail, std::vector <int> ncc_arr_fail, int count_limit)
{


    float STAPLE_Accuracy = accumulate(staple_jacc.begin(), staple_jacc.end(), decltype(staple_jacc)::value_type(0)) * 100.0 / ((float)count_limit);
    float MOSSE_Accuracy = accumulate(mosse_jacc.begin(), mosse_jacc.end(), decltype(mosse_jacc)::value_type(0)) * 100.0 / ((float)count_limit);
    float NCC_Accuracy = accumulate(ncc_jacc.begin(), ncc_jacc.end(), decltype(ncc_jacc)::value_type(0)) * 100.0 / ((float)count_limit);

    cout << "\nSTAPLE Tracker Overlap Accuracy " << STAPLE_Accuracy << " %" << endl;
    cout << "MOSSE Tracker Overlap Accuracy " << MOSSE_Accuracy << " %" << endl;
    cout << "NCC Tracker Overlap Accuracy " << NCC_Accuracy << " %" << endl;

    int staple_fail = accumulate(staple_arr_fail.begin(), staple_arr_fail.end(), decltype(staple_arr_fail)::value_type(0));
    int mosse_fail = accumulate(mosse_arr_fail.begin(), mosse_arr_fail.end(), decltype(mosse_arr_fail)::value_type(0));
    int ncc_fail = accumulate(ncc_arr_fail.begin(), ncc_arr_fail.end(), decltype(ncc_arr_fail)::value_type(0));


    float staple_fail_perc = (float)staple_fail / (float)(count_limit) * 100.0;
    float mosse_fail_perc = (float)mosse_fail / (float)(count_limit) * 100.0;
    float ncc_fail_perc = (float)ncc_fail / (float)(count_limit) * 100.0;

    cout << "\nNumber of STAPLE failures " << staple_fail << "/" << count_limit << " = " << staple_fail_perc << " %" << endl;
    cout << "Number of MOSSE failures " << mosse_fail << "/" << count_limit << " = " << mosse_fail_perc << " %" << endl;
    cout << "Number of NCC failures " << ncc_fail << "/" << count_limit << " = " << ncc_fail_perc << " %" << endl;
}

bool input_params()
{
    char src;
    bool valid_input;

    do
    {
        cout << "Choose to run tracking on Video? [y/N]" << endl;
        cin >> src;

        if (src == 'y' || src == 'Y')
        {
            valid_input = true;
            return true;
        }
        else if (src == 'n' || src == 'N')
        {
            valid_input = true;
            return false;
        }
        else
            valid_input = false;

    } while (valid_input == false);

}

int main()
{
    std::cout << "Hello World!\n";
    bool input_source = input_params(); // true for video, false for frames with ground truth
    string path1 = folder_path(input_source);
    int count_limit = 0;
    VideoCapture cap;

    float IoU_staple = 0.0;
    float IoU_mosse = 0.0;
    float IoU_NCC = 0.0;

    int staple_fail = 0;
    int mosse_fail = 0;
    int ncc_fail = 0;

    Rect gt_result;

    Mat frame_bgr, frame_gray, frame_gray_clone, frame_bgr_clone, frame_bgr_annot;
    Mat template_chosen, template_chosen_bgr;
    bool detector_flag = false;
    int frame_id = 0;
    int detection_cnt = 0;

    float staple_score = 0.0f;
    float mosse_score = 0.0f; 
    float ncc_score = 0.0f;

    Point template_coord;

    bool NCC_occl_flag;
    int Track_status;
    bool Staple_occl_flag;

    int object_state = 1; //Visible by default

    Rect Staple_result;
    Rect MOSSE_result;
    Rect NCC_result;

    STAPLE_TRACKER tracker_staple;
    Tracker_MOSSE tracker_mosse;
    Tracker_NCC tracker_ncc;

    Rect roi;

    double staple_dist = 0.0;
    double mosse_dist = 0.0;
    double ncc_dist = 0.0;

    bool gt_init = false;

    ofstream myfile;

    if (input_source == false) // source is image frames with ground truth data
    {
        count_limit = cnt_files(path1);

        std::filesystem::remove(path1 + "plot.csv"); // delete file
        std::filesystem::remove(path1 + "thresh_plot.csv"); // delete file

        myfile.open(path1 + "plot.csv", std::ofstream::out | std::ofstream::app);
        myfile << "#Frame No., #STAPLE_Jaccard, #MOSSE_Jaccard, #NCC_Jaccard, #STAPLE_Euclid, #MOSSE_Euclid, #NCC_Euclid, ";
        myfile << "#GT_Reinit, #Reinit_decision, #Object_State \n";
    }
    else
    {   
        cap.open(path1); //1 for USB camera, 0 for webcam, path for saved video
        count_limit = (int) (cap.get(CAP_PROP_FRAME_COUNT));
    }


    std::vector <float> staple_arr_thresh(count_limit);
    std::vector <float> mosse_arr_thresh(count_limit);
    std::vector <float> ncc_arr_thresh(count_limit);

    std::vector <int> staple_arr_fail(count_limit);
    std::vector <int> mosse_arr_fail(count_limit);
    std::vector <int> ncc_arr_fail(count_limit);

        while (frame_id < count_limit) // main loop where most of the tracking code is executed
        {
            if (input_source == false) // source is image frames with ground truth data
            {
                frame_bgr = imread(path1 + "v1__0" + "/frame" + "/v1_" + to_string(frame_id) + ".jpg", IMREAD_COLOR);
                gt_result = Ground_truth(path1, frame_id, &object_state);
            }
            else // source is video
            {
                cap >> frame_bgr;
            }

            cvtColor(frame_bgr, frame_gray, COLOR_BGR2GRAY); // Convert scale from BGR -> GrayScale
            frame_bgr_clone = frame_bgr.clone();
            frame_bgr_annot = frame_bgr.clone();
            frame_gray_clone = frame_gray.clone(); // make a clone of the acquired frame
            detector_flag = false;
            frame_bgr.release();
         
                        
            if ((waitKey(1) == 'r') || frame_id == 0) // To Choose a new template on pressing 'r' on keyboard or at frame_id start
            {
                roi = input_source ? selectROI("tracker", frame_gray_clone) : Ground_truth(path1, frame_id, &object_state);
                template_chosen = frame_gray_clone(roi);
                template_chosen_bgr = frame_bgr_clone(roi);
                template_coord = { roi.x, roi.y };
                detector_flag = true; // Tracker may be initialized
                destroyAllWindows();
                tracker_mosse.occlusion_flag_ssd = false;
            }


            ///////////////////////STAPLE Tracker/////////////////////////////
            staple_score = tracker_staple.STAPLE_tracker(frame_bgr_clone, template_chosen_bgr, roi, &Staple_result, detector_flag, &Staple_occl_flag);
            //////////////////////////////////////////////////////////////////
             
            ///////////////////////MOSSE Tracker//////////////////////////////  
            mosse_score = tracker_mosse.MOSSE_TRACKER(template_chosen, template_coord, frame_gray_clone, frame_id, detection_cnt, detector_flag, &MOSSE_result, &Track_status);
            //////////////////////////////////////////////////////////////////

            ///////////////////////Template Match (NCC) Tracker///////////////            
            ncc_score = tracker_ncc.NCC_tracker(frame_gray_clone, template_chosen, roi, detector_flag, &NCC_occl_flag, &NCC_result);
            //////////////////////////////////////////////////////////////////

            Rect Intersection, Union;
            float Jaccard_idx_staple = 0.0, Jaccard_idx_mosse = 0.0, Jaccard_idx_ncc = 0.0;
            Mat staple_cen, mosse_cen, ncc_cen, gt_cen;

            staple_cen = { Staple_result.x + Staple_result.width / 2 ,Staple_result.y + Staple_result.height / 2 };
            mosse_cen = { MOSSE_result.x + MOSSE_result.width / 2 ,MOSSE_result.y + MOSSE_result.height / 2 };
            ncc_cen = { NCC_result.x + NCC_result.width / 2 ,NCC_result.y + NCC_result.height / 2 };

            gt_cen = { gt_result.x + gt_result.width / 2 ,gt_result.y + gt_result.height / 2 };

            if (!Staple_occl_flag) // successful tracking STAPLE Tracker
            {
                rectangle(frame_bgr_annot, Staple_result, Scalar(0, 0, 255), 2); //red box for STAPLE
                putText(frame_bgr_annot, "S", Point(Staple_result.x, Staple_result.y), 1, 1.5, Scalar(0, 0, 255), 2);

                Intersection = gt_result & Staple_result;
                Union = gt_result | Staple_result;
                Jaccard_idx_staple = (float)(Intersection.width * Intersection.height) / (Union.width * Union.height);
                IoU_staple = (Jaccard_idx_staple > IoU_thresh) ? (IoU_staple + Jaccard_idx_staple) : IoU_staple;
                staple_fail = (Jaccard_idx_staple > IoU_thresh) ? 0 : 1;
                staple_dist = norm(staple_cen, gt_cen, NORM_L2);
            }
            else // unsuccessful tracking (occlusion)
            {
                rectangle(frame_bgr_annot, Staple_result, Scalar(255, 255, 255), 2); //red box for STAPLE
                putText(frame_bgr_annot, "S_O", Point(Staple_result.x, Staple_result.y), 1, 1.5, Scalar(0, 0, 255), 2);

                staple_fail = 1;
                Jaccard_idx_staple = 0.0;
            }

            if (Track_status == 2) // successful tracking MOSSE Tracker
            {
                rectangle(frame_bgr_annot, MOSSE_result, Scalar(255, 0, 0), 2);  //blue box for MOSSE
                putText(frame_bgr_annot, "M", Point(MOSSE_result.x, MOSSE_result.y), 1, 1.5, Scalar(255, 0, 0), 2);

                Intersection = gt_result & MOSSE_result;
                Union = gt_result | MOSSE_result;
                Jaccard_idx_mosse = (float)(Intersection.width * Intersection.height) / (Union.width * Union.height);
                IoU_mosse = (Jaccard_idx_mosse > IoU_thresh) ? (IoU_mosse + Jaccard_idx_mosse) : IoU_mosse;
                mosse_fail = (Jaccard_idx_mosse > IoU_thresh) ? 0 : 1;
                mosse_dist = norm(mosse_cen, gt_cen, NORM_L2);
            }
            else // unsuccessful tracking (occlusion)
            {
                rectangle(frame_bgr_annot, MOSSE_result, Scalar(255, 255, 255), 2);  //blue box for MOSSE
                putText(frame_bgr_annot, "M_O", Point(MOSSE_result.x, MOSSE_result.y), 1, 1.5, Scalar(255, 0, 0), 2);

                mosse_fail = 1;
                Jaccard_idx_mosse = 0.0;
            }

            if (NCC_occl_flag != 1)// successful tracking Match Template (NCC) Tracker
            {
                rectangle(frame_bgr_annot, NCC_result, Scalar(0, 255, 0), 2);    //green box for NCC
                putText(frame_bgr_annot, "N", Point(NCC_result.x, NCC_result.y), 1, 1.5, Scalar(0, 255, 0), 2);

                Intersection = gt_result & NCC_result;
                Union = gt_result | NCC_result;
                Jaccard_idx_ncc = (float)(Intersection.width * Intersection.height) / (Union.width * Union.height);
                IoU_NCC = (Jaccard_idx_ncc > IoU_thresh) ? (IoU_NCC + Jaccard_idx_ncc) : IoU_NCC;
                ncc_fail = (Jaccard_idx_ncc > IoU_thresh) ? 0 : 1;
                ncc_dist = norm(ncc_cen, gt_cen, NORM_L2);

            }
            else // unsuccessful tracking (occlusion)
            {
                rectangle(frame_bgr_annot, NCC_result, Scalar(255, 255, 255), 2);    //white box for NCC
                putText(frame_bgr_annot, "N_O", Point(NCC_result.x, NCC_result.y), 1, 1.5, Scalar(0, 255, 0), 2);
                                
                ncc_fail = 1;
                Jaccard_idx_ncc = 0.0;
            }

            if (input_source == false) // source is image frames with ground truth data
            {
                rectangle(frame_bgr_annot, gt_result, Scalar(0, 255, 255), 2);  //yellow box for Ground Truth
                putText(frame_bgr_annot, "GT", Point(gt_result.x, gt_result.y), 1, 1.5, Scalar(0, 255, 255), 2);
                putText(frame_bgr_annot, "Frame ID " + to_string(frame_id), Point(25, 100), 1, 1.5, Scalar(128, 128, 128), 2);
                imshow("Accumulated Result", frame_bgr_annot);

                string Reinit_decision = detector_flag ? "YES" : "NO";

                string Object_state_str =   (object_state == 1) ? "Visible"         : (object_state == 2) ? "25% Occlude" :
                                            (object_state == 3) ? "50% Occlude"     : (object_state == 4) ? "75% Occlude" :
                                            (object_state == 5) ? "100% Occlude"    : (object_state == 6) ? "45° Rotation" :
                                            (object_state == 7) ? "90° Rotation"    : "NA";
                
                gt_init = detector_flag;

                staple_arr_thresh[frame_id] = Jaccard_idx_staple;
                mosse_arr_thresh[frame_id] = Jaccard_idx_mosse;
                ncc_arr_thresh[frame_id] = Jaccard_idx_ncc;

                staple_arr_fail[frame_id] = staple_fail;
                mosse_arr_fail[frame_id] = mosse_fail;
                ncc_arr_fail[frame_id] = ncc_fail;

                myfile << std::to_string(frame_id) + ',';
                myfile << std::to_string(Jaccard_idx_staple) + ',' + std::to_string(Jaccard_idx_mosse) + ',' + std::to_string(Jaccard_idx_ncc) + ',';
                myfile << std::to_string(staple_dist) + ',' + std::to_string(mosse_dist) + ',' + std::to_string(ncc_dist) + ',';
                myfile << std::to_string(gt_init) + ',' + Reinit_decision + ',' + Object_state_str + ',';
                myfile << '\n';
            }
            else
            {
                imshow("Accumulated Result", frame_bgr_annot);
            }

            frame_id++; // increment frame_id upon each frame acquired

            /*if (waitKey(1) == 'q')
                break; */              
        }

        if (input_source == false) // source is image frames with ground truth data
        {
            myfile.close();
            print_tracker_perf(staple_arr_thresh, mosse_arr_thresh, ncc_arr_thresh, staple_arr_fail, mosse_arr_fail, ncc_arr_fail, count_limit);
            plot_graph(path1, count_limit, object_state);
            plot_overlap_threshold(path1, staple_arr_thresh, mosse_arr_thresh, ncc_arr_thresh, count_limit);
        }

    return 0;

    }

