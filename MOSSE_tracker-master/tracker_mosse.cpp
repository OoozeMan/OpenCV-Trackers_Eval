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

#include "tracker_mosse.h"

const int history_limit = 50;
struct Frame_History Hist[history_limit];
float PSR_original = 1.0;

Tracker_MOSSE::Tracker_MOSSE()
{
    //Var initialization
    this->state_ = FOUND;
    this->PSR_mask = 11; //default 11
    this->PSR_ratio[0] = 5; // default 5
    this->PSR_ratio[1] = 9; // default 9

    this->_learning = 0.125; //0.125 default
    this->_im_size.width = 0;
    this->_im_size.height = 0;

    this->_init = false;
    this->_eps = true;

    this->PSR_value = 0.0;
    this->PSR_value_old = 0.0;
    this->frame_id = 0;
    this->ssd_flag = 0;
    this->occlusion_flag = 0;
    this->match_templateVal = 0;
    this->Occlusion_cnt = 0;

    this->blended_template = 0;
    this->Canny_score_orig = 0.0;
    this->tracking_cnt = 0;


}

Tracker_MOSSE::~Tracker_MOSSE()
{


}


void Tracker_MOSSE::InitTracker(const Mat &input_image, Rect input_rect, Rect curr_roi)
{//Init tracker from user selection

   // if (this->_init) return;
    Mat contours;

    if(input_image.empty())
    {
        cout<<"Error while selecting target !"<<endl;
        return;
    }

    this->_im_size.width = input_image.cols;
    this->_im_size.height = input_image.rows;

    prev_img.real_image = input_image(input_rect).clone();

    this->prev_img.cols = this->prev_img.real_image.cols;
    this->prev_img.rows = this->prev_img.real_image.rows;

    ComputeDFT(this->prev_img, true); //Compute Direct Fourier Transform
    SetRoi(curr_roi);
    
    InitFilter();                     //Init filter from ROI (ROI + affine transf)

    this->_init = true;
}

void Tracker_MOSSE::SetRoi(Rect input_ROI)
{//Init ROI position and center

    this->current_ROI.ROI = input_ROI;
    this->current_ROI.ROI_center.x = round(input_ROI.width/2);
    this->current_ROI.ROI_center.y = round(input_ROI.height/2);
}

void Tracker_MOSSE::Track(const Mat &input_image)
{//Perform tracking over current frame

    int cen_x, cen_y;

    if (!this->_init) return;

    if (this->_filter.empty())
    {
        cout << "Error, must initialize tracker first ! "<< endl;
        return;
    }

    Point new_loc;
    Mat input_image_cpy = input_image.clone();

    this->current_img.real_image = input_image(this->current_ROI.ROI).clone(); //Original
    ComputeDFT(this->current_img, true);                    //Compute Direct Fourier Transform

    new_loc = PerformTrack();                                                   //Perform tracking

    if(new_loc.x>=0 && new_loc.y>=0)                                            //If PSR > ratio then update
    {
        this->state_ = FOUND;
        Update(new_loc);                                                        //Update Tracker
    } 
    else
    {
        this->state_ = OCCLUDED;
    }


}

void Tracker_MOSSE::ComputeDFT(image_track &input_image, bool preprocess)
{//Compute Direct Fourier Transform on input image, with or without a pre-processing

    Mat res = this->ComputeDFT(input_image.real_image, preprocess);

    input_image.image_spectrum = res;
    input_image.opti_dft_comp_rows = res.rows;
    input_image.opti_dft_comp_cols = res.cols;
}

Mat Tracker_MOSSE::ComputeDFT(const Mat &input_image, bool preprocess)
{//Compute Direct Fourier Transform on input image, with or without a pre-processing

    Mat gray_padded, complexI;

    int x = input_image.rows;
    int y = input_image.cols;

    //Get optimal dft image size
    int i = getOptimalDFTSize(x);
    int j = getOptimalDFTSize(y);

    //Get optimal dct image size
    //int i = 2*getOptimalDFTSize((x+1)/2);
    //int j = 2*getOptimalDFTSize((y+1)/2);

    //Zero pad input image up to optimal dft size
    copyMakeBorder(input_image,gray_padded,0,i-x,0,j-y,BORDER_CONSTANT,Scalar::all(0));

    input_image.copyTo(gray_padded);

    //If input image is RGB, convert to gray scale
    if (gray_padded.channels() > 1) cvtColor(gray_padded,gray_padded, COLOR_BGR2GRAY);
    gray_padded.convertTo(gray_padded,CV_32F);

    if (preprocess)     //Apply pre-processing to input image
    { 

        // DCT Stuff
//        cv::dct(gray_padded, gray_padded, CV_DXT_FORWARD);
//        for (int i=0; i<gray_padded.rows; i+=1)
//        {
//            for (int j=0; j<gray_padded.cols; j+=1)
//            {
//                gray_padded.at<float>(i,j) *= (1 / (1 + (exp(-(i * gray_padded.cols + j)))));
//            }
//        }
//        gray_padded.at<float>(0,0) = 0;
//        cv::dct(gray_padded, gray_padded, CV_DXT_INVERSE);


        cv::normalize(gray_padded, gray_padded, 0.0, 1.0, NORM_MINMAX);

        gray_padded += Scalar::all(1);
        cv::log(gray_padded, gray_padded);

        cv::Scalar mean,stddev;
        cv::meanStdDev(gray_padded,mean,stddev);
        gray_padded -= mean.val[0];

        cv::Mat tmp;
        cv::multiply(gray_padded, gray_padded, tmp);
        cv::Scalar sum_ = cv::sum(tmp);
        gray_padded /= sum_.val[0];

        //Apply Hanning window to reduce image boundaries effect
        if (this->_HanningWin.empty() || gray_padded.size() != this->_HanningWin.size())
        {
            cv::Mat hanningWin_;
            cv::createHanningWindow(hanningWin_, gray_padded.size(), CV_32F);
            hanningWin_.copyTo(this->_HanningWin);
        }

        cv::multiply(gray_padded, this->_HanningWin, gray_padded);

    }

    dft(gray_padded, complexI, DFT_COMPLEX_OUTPUT);    //Compute Direct Fourier Transform

    //Crop the spectrum, if it has an odd number of rows or columns
    complexI = complexI(Rect(0, 0, complexI.cols & -2, complexI.rows & -2));

    return complexI;
}

Point Tracker_MOSSE::PerformTrack()
{
    Mat mat_correlation,idft_correlation;
    Mat mat_correlation_old, idft_correlation_old;
    float PSR_val, PSR_val_old;
    Point maxLoc;
    //Element-wise matrice multiplication, second arg is complex conjugate H*
    mulSpectrums(this->current_img.image_spectrum,this->_filter,mat_correlation,0,false);
    mulSpectrums(this->current_img.image_spectrum, this->_filterOld, mat_correlation_old, 0, false);

    //Inverse DFT real output
    dft(mat_correlation, idft_correlation, DFT_INVERSE | DFT_REAL_OUTPUT);
    dft(mat_correlation_old, idft_correlation_old, DFT_INVERSE | DFT_REAL_OUTPUT);

    // ***************************************************************************************************
    /* ---------------------------------------------------- */
    // Showing the images in the GUI

    // Correlation image
    Mat corrImg;
    normalize(idft_correlation, idft_correlation, 0.0, 1.0, NORM_MINMAX);
    resize(idft_correlation, corrImg, cv::Size(136,79)); // not taking into account a resized window!

    normalize(idft_correlation_old, idft_correlation_old, 0.0, 1.0, NORM_MINMAX);

    // Filter image
    Mat filtImg, filtImg_old;
    dft(this->_filter, filtImg, DFT_INVERSE | DFT_REAL_OUTPUT); //
    int cx = filtImg.cols/2;
    int cy = filtImg.rows/2;
    Mat q0(filtImg, Rect(0, 0, cx, cy));   // Top-Left - Create a ROI per quadrant
    Mat q1(filtImg, Rect(cx, 0, cx, cy));  // Top-Right
    Mat q2(filtImg, Rect(0, cy, cx, cy));  // Bottom-Left
    Mat q3(filtImg, Rect(cx, cy, cx, cy)); // Bottom-Right
    Mat tmp;                           // swap quadrants (Top-Left with Bottom-Right)
    q0.copyTo(tmp);
    q3.copyTo(q0);
    tmp.copyTo(q3);
    q1.copyTo(tmp);                    // swap quadrant (Top-Right with Bottom-Left)
    q2.copyTo(q1);
    tmp.copyTo(q2);

    normalize(filtImg, filtImg, 0.0, 1.0, NORM_MINMAX);
    resize(filtImg, filtImg, cv::Size(136,79)); // not taking into account a resized window!
    flip(filtImg,filtImg,0);

    // Input image
    cv::Mat input_;
    this->current_img.real_image.copyTo(input_);
    cv::resize(input_, input_, cv::Size(136,79));

    //emit newCorrelationImage(CImage::getQImage(corrImg));
    //emit newFilterImage(CImage::getQImage(filtImg));
    //emit newInputImage(CImage::getQImage(input_));
    /* ----------------------------------------------------- */
    // ***************************************************************************************************

    PSR_val = ComputePSR(idft_correlation);  //Compute PSR
    PSR_val_old = ComputePSR(idft_correlation_old);  //Compute PSR against original filter


    if ((PSR_val_old <= PSR_ratio[1]) || (PSR_val_old < PSR_ratio[0]))
    {
        this->Occlusion_cnt++;
        this->tracking_cnt = 0;
    }
    else
    {       
        this->Occlusion_cnt = 0;
        this->tracking_cnt++;
    }

    int tracking_mod = this->tracking_cnt % 150;

    if (tracking_mod == 1) //Update filter after every x-frames
    {
        this->_filterOld = this->_filter.clone();
    }

    this->PSR_value = PSR_val;
    this->PSR_value_old = PSR_val_old;

    if (PSR_val >= PSR_ratio[1])    //Get new pos if object detected
        {

            minMaxLoc(idft_correlation,NULL,NULL,NULL,&maxLoc);

            // ******* Trying to keep a nice and clean filter output image ***********************
            cv::Mat new_output = cv::Mat::zeros(mat_correlation.size(), CV_32F);

            MaskDesiredG(new_output,maxLoc.x,maxLoc.y);

            new_output = ComputeDFT(new_output, false);
            // ***********************************************************************************

            new_output.copyTo(this->current_img.filter_output);

        }else if(PSR_val > PSR_ratio[0]){ //Return -1 coordinates if object occluded
            maxLoc.x = -1;
            maxLoc.y = -1;
        }else{                           //Return -2 coordinates if object lost
            maxLoc.x = -2;
            maxLoc.y = -2;
        }

    return maxLoc;
}

float Tracker_MOSSE::ComputePSR(const Mat &correlation_mat)
{//Compute Peak-to-Sidelobe Ratio

    double max_val = 0.0;
    Point max_loc;
    Mat PSR_mask = Mat::ones(correlation_mat.rows,correlation_mat.cols, CV_8U);
    Scalar mean,stddev;

    minMaxLoc(correlation_mat,NULL,&max_val,NULL,&max_loc);     //Get location of max arg

    //Define PSR mask
    int win_size = floor(this->PSR_mask/2);
    Rect mini_roi = Rect(std::max(max_loc.x - win_size,0), std::max(max_loc.y - win_size,0), this->PSR_mask, this->PSR_mask);

    //Handle image boundaries
    if ( (mini_roi.x+mini_roi.width) > PSR_mask.cols )
    {
        mini_roi.width = PSR_mask.cols - mini_roi.x;
    }
    if ( (mini_roi.y+mini_roi.height) > PSR_mask.rows )
    {
        mini_roi.height = PSR_mask.rows - mini_roi.y;
    }


    PSR_mask(mini_roi) = 0;

    meanStdDev(correlation_mat,mean,stddev,PSR_mask);   //Compute matrix mean and std
         
    return (max_val - mean.val[0]) / stddev.val[0];     //Compute PSR
}

void Tracker_MOSSE::UpdateRoi(Point new_center, bool scale_rot)
{//Update ROI position

    int diff_x,diff_y;

    //Current ROI pos is previous ROI pos
    this->prev_ROI = this->current_ROI;

//    if (scale_rot)
//    {
//        //ComputeOrientationScale();
//    }

    //Define new ROI position
    this->current_ROI.ROI_center = new_center;

    new_center.x += prev_ROI.ROI.x ;
    new_center.y += prev_ROI.ROI.y ;

    //Handle image boundarie
    diff_x = new_center.x - round(this->current_ROI.ROI.width/2);
    diff_y = new_center.y - round(this->current_ROI.ROI.height/2);

    if (diff_x < 0)
    {
        this->current_ROI.ROI.x = 0;
    }
    else if( (diff_x + this->current_ROI.ROI.width) >= this->_im_size.width )
    {
        this->current_ROI.ROI.x = this->_im_size.width - this->current_ROI.ROI.width -1;
    }else{
        this->current_ROI.ROI.x = diff_x;
    }

    if (diff_y < 0)
    {
        this->current_ROI.ROI.y = 0;
    }
    else if( (diff_y + this->current_ROI.ROI.height) >= this->_im_size.height )
    {
        this->current_ROI.ROI.y = this->_im_size.height - this->current_ROI.ROI.height -1;
    }else{
        this->current_ROI.ROI.y = diff_y;
    }

    this->current_ROI.ROI.width = this->prev_ROI.ROI.width;
    this->current_ROI.ROI.height = this->prev_ROI.ROI.height;
}

void Tracker_MOSSE::Update(Point new_location)
{//Update Tracker_MOSSE

    UpdateFilter();                         //Update filter
    this->prev_img = this->current_img;     //Update frame (default)
    UpdateRoi(new_location, false);          //Update ROI position

    //imshow("Updating Template", this->prev_img.real_image);
}

void Tracker_MOSSE::InitFilter()
{//Init filter from user selection

    Mat affine_G, affine_image, temp_image_dft, temp_desi_G, filter;
    Mat temp_FG,temp_FF, num, dem, eps;

    //Number of images to init filter
    int N = 8;

    //Create the the desired output - 2D Gaussian
    Mat Mask_gauss = cv::Mat::zeros(this->prev_img.real_image.size(), CV_32F);
    MaskDesiredG(Mask_gauss,round(this->current_ROI.ROI.width/2),
                 round(this->current_ROI.ROI.height/2));


    temp_FG = Mat::zeros(this->prev_img.opti_dft_comp_rows,this->prev_img.opti_dft_comp_cols,this->prev_img.image_spectrum.type());
    temp_FF = Mat::zeros(this->prev_img.opti_dft_comp_rows,this->prev_img.opti_dft_comp_cols,this->prev_img.image_spectrum.type());

    temp_image_dft = this->prev_img.image_spectrum;
    temp_desi_G = ComputeDFT(Mask_gauss, false);

    temp_desi_G.copyTo(this->prev_img.filter_output);

    mulSpectrums(temp_desi_G, temp_image_dft,num,0,true);       //Element-wise spectrums multiplication G o F*
    temp_FG += num;

    mulSpectrums(temp_image_dft,temp_image_dft,dem,0,true);     //Element-wise spectrums multiplication F o F*
    temp_FF += dem;

    if (_eps)
    {
        //Regularization parameter
        eps = createEps(dem);
        dem += eps;
        temp_FF += dem;
    }

    srand(time(NULL));

    for (int i=0;i<(N-1);i++)
    {//Create image dataset with input image affine transforms

        AffineTransform(Mask_gauss,this->prev_img.real_image,affine_G,affine_image);    //Input image and desired output affine transform

        temp_image_dft = ComputeDFT(affine_image, true);        //Affine image DFT
        temp_desi_G = ComputeDFT(affine_G, false);              //Affine output DFT

        mulSpectrums(temp_desi_G, temp_image_dft,num,0,true);   //Element-wise spectrums multiplication G o F*
        temp_FG += num;

        mulSpectrums(temp_image_dft,temp_image_dft,dem,0,true); //Element-wise spectrums multiplication F o F*

        if (_eps)
        {
            eps = createEps(dem);
            dem += eps;
        }

        temp_FF += dem;
    }

    dftDiv(temp_FG, temp_FF, filter);       //Element-wise spectrum Division

    filter.copyTo(this->_filter);           //Filter

    this->_filterOld = filter.clone();        //Save a copy of original Filter

    //filter = conj(this->_filter);
    //inverseAndSave(filter, "filter_inv_shift.jpg", true);
}

void Tracker_MOSSE::AffineTransform(const Mat &input_image, const Mat &input_image2, Mat &aff_img, Mat &aff_img2)
{//Apply same randomly defined affine transform to both input matrice

    if (input_image.size() != input_image2.size())
    {
        cout<<"Error while computing affine transform !"<<endl;
        return;
    }

    //output images
    aff_img = Mat::zeros(input_image.rows,input_image.cols,input_image.type());
    aff_img2 = Mat::zeros(input_image2.rows,input_image2.cols,input_image2.type());

    int cols = input_image.cols;
    int rows = input_image.rows;

    Point2f input_pts[3];
    Point2f output_pts[3];

    float pts0_r, pts0_c, pts1_r, pts1_c, pts2_r, pts2_c;

    Mat affine_tr(2,3,CV_32FC1);

    input_pts[0] = Point2f(0,0);
    input_pts[1] = Point2f(cols - 1, 0);
    input_pts[2] = Point2f(0, rows - 1);

    //Define affine transform 'intensity'
    pts0_r = rand() % 5; pts0_r /= 100;
    pts0_c = rand() % 5; pts0_c /= 100;

    pts1_r = rand() % 5; pts1_r /= 100;
    pts1_c = rand() % 5 + 95; pts1_c /= 100;

    pts2_r = rand() % 5 + 95; pts2_r /= 100;
    pts2_c = rand() % 5; pts2_c /= 100;

    output_pts[0] = Point2f(cols*pts0_c,rows*pts0_r);
    output_pts[1] = Point2f(cols*pts1_c,rows*pts1_r);
    output_pts[2] = Point2f(cols*pts2_c,rows*pts2_r);

    affine_tr = getAffineTransform( input_pts, output_pts );        //Get transformation matrix

    warpAffine( input_image, aff_img, affine_tr, aff_img.size() );  //Apply transformation matrix
    warpAffine( input_image2, aff_img2, affine_tr, aff_img2.size() );
}

void Tracker_MOSSE::MaskDesiredG(Mat &output,int u_x,int u_y,double sigma, bool norm_energy)
{//Create 2D Gaussian

    sigma *= sigma;

    //Fill input matrix as 2D Gaussian
    for(int i=0;i<output.rows;i++)
    {
        for(int j=0;j<output.cols;j++)
        {
            output.at<float>(i,j) = 255 * exp( (-(i-u_y)*(i-u_y) / (2*sigma)) +
                                     (-(j-u_x)*(j-u_x) / (2*sigma)) );
        }
    }

    if (norm_energy)    //If true, norm image energy so that it sum up to 1
    {
        Scalar sum_;
        sum_ = sum(output);
        output /= sum_.val[0];
    }

}

void Tracker_MOSSE::UpdateFilter()
{//Update filter
    Mat Ai,Bi,Ai_1,Bi_1,A,B,filter,eps,eps_1;

    mulSpectrums(this->current_img.filter_output, this->current_img.image_spectrum,Ai,0,true);      //Element-wise spectrums multiplication G o F*
    mulSpectrums(this->prev_img.filter_output, this->prev_img.image_spectrum,Ai_1,0,true);          //Element-wise spectrums multiplication G-1 o F-1*

    mulSpectrums(this->current_img.image_spectrum, this->current_img.image_spectrum,Bi,0,true);     //Element-wise spectrums multiplication F o F*
    mulSpectrums(this->prev_img.image_spectrum, this->prev_img.image_spectrum,Bi_1,0,true);         //Element-wise spectrums multiplication F-1 o F-1*

    if (_eps)
    {
        //Regularization parameter
        eps = createEps(Bi);
        Bi += eps;

        eps_1 = createEps(Bi_1);
        Bi_1 += eps;
    }


    // MOSSE update

    A = ( ((1.0-_learning)*Ai) + ((_learning)*Ai_1) );
    B = ( ((1.0-_learning)*Bi) + ((_learning)*Bi_1) );
    dftDiv(A, B, filter);
    filter.copyTo(this->_filter);


    // ASEF update

//    dftDiv(Ai, Bi, A);
//    filter = (A * (1.0 - _learning)) - (this->_filter * _learning);
//    filter.copyTo(this->_filter);

}

void Tracker_MOSSE::inverseAndSave(const cv::Mat &img, const std::string &filename, const bool &shift)
{//Inverse DFT and save image

    cv::Mat img_i;

    cv::dft(img, img_i, DFT_INVERSE | DFT_REAL_OUTPUT);

    if (shift)      //If true swap quadrants
    {
        int cx = img_i.cols/2;
        int cy = img_i.rows/2;
        Mat q0(img_i, Rect(0, 0, cx, cy));   // Top-Left - Create a ROI per quadrant
        Mat q1(img_i, Rect(cx, 0, cx, cy));  // Top-Right
        Mat q2(img_i, Rect(0, cy, cx, cy));  // Bottom-Left
        Mat q3(img_i, Rect(cx, cy, cx, cy)); // Bottom-Right
        Mat tmp;                             // swap quadrants (Top-Left with Bottom-Right)
        q0.copyTo(tmp);
        q3.copyTo(q0);
        tmp.copyTo(q3);
        q1.copyTo(tmp);                      // swap quadrant (Top-Right with Bottom-Left)
        q2.copyTo(q1);
        tmp.copyTo(q2);
    }

    cv::normalize(img_i, img_i, 0.0, 255.0, NORM_MINMAX);
    cv::imwrite(filename, img_i);
}

Mat Tracker_MOSSE::conj(const Mat &input_dft)
{//Compute complex conjugate

    assert (input_dft.channels() == 2);

    Mat conj_dft;

    input_dft.copyTo(conj_dft);

    //Invert imaginary part sign
    for (int x=0;x<input_dft.rows;x++)
    {
        for (int y=0;y<input_dft.cols;y++)
        {
            conj_dft.at<cv::Vec2f>(x,y)[1] *= -1.0;
        }
    }

    return conj_dft;
}

void Tracker_MOSSE::dftDiv(const Mat &dft_a, const Mat &dft_b, Mat &output_dft)
{//Compute complex divison

    assert (dft_a.size() == dft_b.size() && dft_a.type() == dft_b.type() &&
            dft_a.channels() == dft_b.channels() && dft_a.channels() == 2);

    Mat out_temp = Mat::zeros(dft_a.rows,dft_a.cols,dft_a.type());

    for (int x=0;x<dft_a.rows;x++)
    {
        for (int y=0;y<dft_a.cols;y++)
        {
            out_temp.at<cv::Vec2f>(x,y)[0] = ( (dft_a.at<cv::Vec2f>(x,y)[0] * dft_b.at<cv::Vec2f>(x,y)[0]) +
                    (dft_a.at<cv::Vec2f>(x,y)[1] * dft_b.at<cv::Vec2f>(x,y)[1]) ) /
                    ( (dft_b.at<cv::Vec2f>(x,y)[0] * dft_b.at<cv::Vec2f>(x,y)[0]) +
                    (dft_b.at<cv::Vec2f>(x,y)[1] * dft_b.at<cv::Vec2f>(x,y)[1]) );

            out_temp.at<cv::Vec2f>(x,y)[1] = ( (dft_a.at<cv::Vec2f>(x,y)[1] * dft_b.at<cv::Vec2f>(x,y)[0]) -
                    (dft_a.at<cv::Vec2f>(x,y)[0] * dft_b.at<cv::Vec2f>(x,y)[1]) ) /
                    ( (dft_b.at<cv::Vec2f>(x,y)[0] * dft_b.at<cv::Vec2f>(x,y)[0]) +
                    (dft_b.at<cv::Vec2f>(x,y)[1] * dft_b.at<cv::Vec2f>(x,y)[1]) );
        }
    }

    out_temp.copyTo(output_dft);
}

Mat Tracker_MOSSE::createEps(const Mat &input_, double std)
{//Compute regularization parameter for a given input matrix

    //Compute input matrix mean and std
    cv::Scalar mean,stddev;
    cv::meanStdDev(input_,mean,stddev);

    Mat eps = Mat::zeros(input_.size(),input_.type());

    //Fill output matrix so that white noise zero mean and std a fraction of input matrix mean value
    randn(eps,0,std*(mean.val[0]));

    //Set imaginary part of noise to all zeros
    for (int x=0;x<eps.rows;x++)
    {
        for (int y=0;y<eps.cols;y++)
        {
            eps.at<cv::Vec2f>(x,y)[1] = 0;
        }
    }

    eps.at<cv::Vec2f>(0,0)[0] = 0;
    eps.at<cv::Vec2f>(input_.rows-1,0)[0] = 0;
    eps.at<cv::Vec2f>(0,input_.cols-1)[0] = 0;
    eps.at<cv::Vec2f>(input_.rows-1,input_.cols-1)[0] = 0;

    return eps;
}

/* ------------------------------------ */
Rect Tracker_MOSSE::getPosition() const       //Get ROI position
{
    return current_ROI.ROI;
}

int Tracker_MOSSE::getState() const
{
    return state_;
}

bool Tracker_MOSSE::isInitialized() const     //Verify tracker is init
{
    return _init;
}

void Tracker_MOSSE::SetPSR_mask(int input)        //Set PSR var
{
    this->PSR_mask = input;
}

void Tracker_MOSSE::SetPSR_ratio_low(int input)
{
    this->PSR_ratio[0] = input;
}

void Tracker_MOSSE::SetPSR_ratio_high(int input)
{
    this->PSR_ratio[1] = input;
}

int Tracker_MOSSE::GetPSR_mask() const        //Get PSR var
{
    return this->PSR_mask;
}

int Tracker_MOSSE::GetPSR_ratio_low() const
{
    return this->PSR_ratio[0];
}

int Tracker_MOSSE::GetPSR_ratio_high() const
{
    return this->PSR_ratio[1];
}

Mat Tracker_MOSSE::GetFilter() const      //Get filter
{
    return this->_filter;
}

float Tracker_MOSSE::Get_Learning() const     //Get/Set learning ratio
{
    return this->_learning;
}

void Tracker_MOSSE::Set_Learning(float val)
{
    this->_learning = val;
}

float Tracker_MOSSE::calculate_ssd(Mat query_frame, Mat template_frame)
{
    if (query_frame.cols != template_frame.cols || query_frame.rows != template_frame.rows)
        return 0;

    Mat ssqd_compute, ssq_compute1, ssq_compute2;
    float ssq_query_sum_of_sq, ssq_template_sum_of_sq;
    float sum_of_square_diff = 0.0;
    float norm_ssqd = 0.0;
    float normalizer = float(query_frame.rows) * float(query_frame.cols) * 1000;

    float min_val = 0.0, max_val = 0.0;
    
    Mat query_float32, template_float32;
    query_frame.convertTo(query_float32, CV_32F);
    template_frame.convertTo(template_float32, CV_32F);

    subtract(query_float32, template_float32, ssqd_compute);
    pow(ssqd_compute, 2, ssqd_compute);
    sum_of_square_diff = sum(ssqd_compute)[0];
   
    pow(query_float32, 2, ssq_compute1);
    pow(template_float32, 2, ssq_compute2);
    ssq_query_sum_of_sq = sum(ssq_compute1)[0];
    ssq_template_sum_of_sq = sum(ssq_compute2)[0];

    normalizer = sqrt(ssq_query_sum_of_sq * ssq_template_sum_of_sq);
    norm_ssqd = 1.00000 - (sum_of_square_diff / (normalizer));
   
    return norm_ssqd;
}

float Tracker_MOSSE::MOSSE_TRACKER(Mat Template, Point Template_Coord, Mat Frame_in, int frame_id, int detection_cnt, bool detection_flag, Rect* Track_result, int* Track_status)
{
    float tracker_score = 0.0;
    const int history_limit = 50;

    int template_height = Template.rows;
    int template_width = Template.cols;
    float PSR_ratio = 0.0;
    float ssd_score = 0.0, edge_score = 0.0;
    Mat Template_match, contours;

    Mat Frame_in_clone = Frame_in.clone();

    double minVal; double maxVal; Point minLoc; Point maxLoc;
    Point matchLoc;

    float hassan_score;
    int maxima_x, maxima_y;
    Rect Result_roi;

    this->frame_id = frame_id;

    Rect roi(Template_Coord.x, Template_Coord.y, template_width, template_height);

    if (detection_flag == true) 
    {
        InitTracker(Frame_in, roi, roi); // Re-Initialize the Tracker with updated Frame and RoI 
        InitHist_Struct(Frame_in, roi, history_limit); // Fill entire structure with copies of Detected frames
    }

    Track(Frame_in);   // Track object in frame against provided parameters 

    *Track_status = this->state_; // FOUND, OCCLUDE or LOST
    *Track_result = this->current_ROI.ROI;  // (x,y,w,h) or updated roi

    int frame_id_mod = frame_id % history_limit;

        PSR_original = PSR_original;

    if (*Track_status == FOUND) // Save parameters in History list
    {
        Hist[frame_id_mod] = Tracker_History(Frame_in, *Track_result, frame_id, tracker_score, *Track_status);
        this->occlusion_flag = false; // occlusion false
    }
    else if ((*Track_status != FOUND) && (this->occlusion_flag == false))
    {
        InitTracker(Hist[frame_id_mod].frame, Hist[frame_id_mod].roi, this->current_ROI.ROI); // Re-Initialize the Tracker with updated Frame and RoI 
        InitHist_Struct(Hist[frame_id_mod].frame, Hist[frame_id_mod].roi, history_limit); // Fill entire structure with copies of Detected frames
        this->occlusion_flag = true; //occlusion true
    }
    
    tracker_score = this->PSR_value;

    return tracker_score;
}

Frame_History Tracker_MOSSE::Tracker_History(Mat frame_in, Rect Roi_current, int frame_id, float Tracker_Score, int tracker_state)
{
    struct Frame_History Hist;

    Hist.frame = frame_in;                              // Saving current frame image in Structure
    Hist.frameID = frame_id;                            // Saving current frame ID in Structure
    Hist.roi = Roi_current;                             // Saving current frame ROI coordinates in Structure
    Hist.score = Tracker_Score;

    return (Hist);

}

void Tracker_MOSSE::InitHist_Struct(Mat frame_init, Rect roi_init, int limit)
{
    for (int i = 0; i < limit; i++) // Fill entire History structure with copies of Detected Frame.
    {
        Hist[i].frame = frame_init;
        Hist[i].frameID = 0;
        Hist[i].roi = roi_init;
        Hist[i].score = 0.0;
    }


}

void Tracker_MOSSE::Annotate_Tracker(Mat frame_in, Rect Roi_current, int tracker_state, float SSD_Score)
{
    int Cen_x = Roi_current.x + Roi_current.width / 2;
    int Cen_y = Roi_current.y + Roi_current.height / 2;
    Point centroid = { Cen_x , Cen_y };

    Mat frame_annot = frame_in.clone();


    if (tracker_state == FOUND)
    {
        rectangle(frame_annot, Roi_current, Scalar(255, 0, 0), 2, 1, 0);
        drawMarker(frame_annot, centroid, Scalar(255, 0, 0), MARKER_CROSS, 50, 2, 8);
        putText(frame_annot, "PSR Value= " + to_string(this->PSR_value), Point(10, 70), FONT_HERSHEY_DUPLEX, 0.5, Scalar(255, 0, 0), 2);
       putText(frame_annot, "SSD Score= " + to_string(SSD_Score), Point(10, 90), FONT_HERSHEY_DUPLEX, 0.5, Scalar(255, 0, 0), 2);
    }

    else if (tracker_state == LOST)
    {
        drawMarker(frame_annot, centroid, Scalar(255, 0, 0), MARKER_TILTED_CROSS, 50, 2, 8);
        putText(frame_annot, "Object Lost", Point(10, 50), FONT_HERSHEY_DUPLEX, 0.5, Scalar(255, 0, 0), 2);
    }

    else if (tracker_state == OCCLUDED)
    {
        drawMarker(frame_annot, centroid, Scalar(255, 0, 0), MARKER_TILTED_CROSS, 50, 2, 8);
        putText(frame_annot, "Object Occluded", Point(10, 50), FONT_HERSHEY_DUPLEX, 0.5, Scalar(255, 0, 0), 2);
        putText(frame_annot, "PSR Value= " + to_string(this->PSR_value), Point(10, 70), FONT_HERSHEY_DUPLEX, 0.5, Scalar(255, 0, 0), 2);
    }


    else
    {
        imshow("Updated Tracking", frame_annot);
    }


    imshow("Updated MOSSE Tracking", frame_annot);
    imshow("MOSSE Template", this->current_img.real_image);

    /// <summary>
    /// /////////////Testing HoG ////////////////
    /// </summary>
    /// <param name="frame_in"></param>
    /// <param name="Roi_current"></param>
    /// <param name="tracker_state"></param>
    /// <param name="SSD_Score"></param>
    //HOGDescriptor hog1, hog2;
    //vector<float> ders1, ders2;
    //vector<Point> locs1, locs2;
    ////resize(img1_gray, r_img1_gray, Size(64, 8));
    //hog1.compute(this->current_img.real_image, ders1, Size(32, 32), Size(0, 0), locs1);
    //hog2.compute(this->prev_img.real_image, ders2, Size(32, 32), Size(0, 0), locs2);

    //Mat Hogfeat1(ders1.size(), 1, CV_32FC1);
    //Mat Hogfeat2(ders2.size(), 1, CV_32FC1);

    //for (int i = 0; i < ders1.size(); i++)
    //    Hogfeat1.at<float>(i, 0) = ders1.at(i);

    //for (int i = 0; i < ders2.size(); i++)
    //    Hogfeat2.at<float>(i, 0) = ders2.at(i);


    //cout << "Hogfeat rows = " << Hogfeat1.rows << endl;

    //// This is for comparing the HOG features of two images without using any SVM 
    //// (It is not an efficient way but useful when you want to compare only few or two images)
    //// Simple distance
    //// Consider you have two HOG feature vectors for two images Hogfeat1 and Hogfeat2 and those are same size.

    //double distance = 0;
    //for (int i = 0; i < Hogfeat1.rows; i++)
    // {
    //    distance += abs(Hogfeat1.at<float>(i, 0) - Hogfeat2.at<float>(i, 0));
    // }


    //cout << "HoG distance is " << distance << endl;
}

float Tracker_MOSSE::Occlusion_Test(Mat curr_template, Mat prev_template)
{
    Mat left_half_curr, right_half_curr;
    Mat left_half_prev, right_half_prev;
    Mat upper_half_curr, lower_half_curr;
    Mat upper_half_prev, lower_half_prev;
    Mat SSD_score;

    float left_h_ssd = 0.0;
    float right_h_ssd = 0.0;
    float upper_h_ssd = 0.0;
    float lower_h_ssd = 0.0;
    float template_score = 0.0;

    double minVal; double maxVal; Point minLoc; Point maxLoc;

    int rows = curr_template.rows;
    int cols = curr_template.cols;

    //cout << "Rows x Cols " << rows << "x" << cols << endl;

    left_half_curr = curr_template({ 0,0,cols / 2,rows}); // Choosing left-half of current template
    right_half_curr = curr_template({(cols / 2) - 1, 0, cols / 2,rows }); // Choosing right-half of previous template

    left_half_prev = prev_template({ 0,0,cols / 2,rows }); // Choosing left-half of previous template
    right_half_prev = prev_template({ (cols / 2) - 1, 0, cols / 2,rows }); // Choosing right-half of previous template

    upper_half_curr = curr_template({ 0,0,cols,rows / 2 }); // Choosing left-half of current template
    lower_half_curr = curr_template({ 0, (rows / 2) - 1, cols , rows / 2 }); // Choosing right-half of previous template

    upper_half_prev = prev_template({ 0,0,cols,rows / 2 }); // Choosing left-half of previous template
    lower_half_prev = prev_template({ 0, (rows / 2) - 1, cols , rows / 2 }); // Choosing right-half of previous template



    //imshow("Left Half Template", left_half_prev);
    //imshow("Right Half Template", right_half_prev);

   left_h_ssd =  calculate_ssd(left_half_curr, left_half_prev);
   right_h_ssd = calculate_ssd(right_half_curr, right_half_prev);

   upper_h_ssd = calculate_ssd(upper_half_curr, upper_half_prev);
   lower_h_ssd = calculate_ssd(lower_half_curr, lower_half_prev);

   template_score = calculate_ssd(curr_template, prev_template);
     
   occlusion_flag_ssd = false;

   const float occl_thresh = 0.99;

   if (left_h_ssd < (occl_thresh * right_h_ssd)) //0.75
   {
       cout << "Stop Template update left " << left_h_ssd << endl;
       occlusion_flag_ssd = true;
   }

   if (right_h_ssd < (occl_thresh * template_score) )
   {
       cout << "Stop Template update right " << right_h_ssd << endl;       
       occlusion_flag_ssd = true;
   }

   if (upper_h_ssd < (occl_thresh * template_score) )
   {
       cout << "Stop Template update upper " << upper_h_ssd << endl;
       occlusion_flag_ssd = true;
   }
  

   if (lower_h_ssd < (occl_thresh * template_score))
   {
       cout << "Stop Template update lower " << lower_h_ssd << endl;
       occlusion_flag_ssd = true;
   }
  
   cout << "Template SSD Score " << template_score << "Left Half " << left_h_ssd << "Right Half " << right_h_ssd << endl;

   return 0.0;
   
}

/* ------------------------------------ */


