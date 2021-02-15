#ifndef IMPROEDGEENHANCEMENT_H
#define IMPROEDGEENHANCEMENT_H

#include <opencv2/opencv.hpp>

void gaussianBlurAndLaplacian(
        cv::InputArray src,
        cv::OutputArray dst,
        cv::Size blurKsize = cv::Size(3, 3),
        double sigmaX = 0.0,
        double sigmaY = 0.0,
        int ddepth = CV_16S,
        int ksize = 3,
        double scale = 1.0,
        double delta = 0.0,
        int borderType = cv::BORDER_DEFAULT
        );

#endif // IMPROEDGEENHANCEMENT_H
