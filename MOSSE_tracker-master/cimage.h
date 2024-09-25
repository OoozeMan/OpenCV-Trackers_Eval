#ifndef CIMAGE_H
#define CIMAGE_H


#include <opencv2/highgui.hpp>

class CImage
{
public:
    CImage();
    CImage(const cv::Mat& mat);
    ~CImage();

    //QImage  getQImage() const;
    cv::Mat getMat() const              { return image_; }
    void    setMat(const cv::Mat & mat) { image_ = mat;  }
   // QImage static getQImage(const cv::Mat& mat);

private:
    cv::Mat image_;
};

#endif // CIMAGE_H
