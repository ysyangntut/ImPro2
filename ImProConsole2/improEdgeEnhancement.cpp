#include <iostream>
#include <cstdio>

#include "improEdgeEnhancement.h"

using std::cerr;
using std::cout;
using std::endl;

void gaussianBlurAndLaplacian(
        cv::InputArray src,
        cv::OutputArray dst,
        cv::Size blurKsize, // = cv::Size(3, 3),
        double sigmaX, // = 0.0,
        double sigmaY, // = 0.0,
        int ddepth, // = CV_16S,
        int ksize, // = 3,
        double scale, // = 1.0,
        double delta, // = 0.0,
        int borderType  // = cv::BORDER_DEFAULT
        )
{
    // check
    if (src.empty()) {
        cerr << "gaussianBlurAndLaplacian: Got an empty source image.\n";
        return;
    }
    // According to OpenCV documentation,  if sigmaY is zero, it is set to be
    // equal to sigmaX, if both sigmas are zeros, they are computed from
    // ksize.width and ksize.height, respectively
    if (sigmaY == 0.0) sigmaY = sigmaX;
    // Reduce noise by blurring with a Gaussian filter ( kernel size = 3 )
    cv::GaussianBlur( src, dst, blurKsize, sigmaX, sigmaY, borderType);
    // Convert to gray
    cv::Mat src_gray;
    if (src.channels() == 3)
        cv::cvtColor( src, src_gray, cv::COLOR_BGR2GRAY ); // Convert the image to grayscale
    else
        src_gray = src.getMat();
    // Laplacian
    cv::Mat laplacianDst;
    cv::Laplacian(src_gray, laplacianDst, ddepth, ksize, scale, delta, borderType);
    // converting back to CV_8U
    cv::convertScaleAbs( laplacianDst, dst);
}
