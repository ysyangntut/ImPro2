// Copyright (c) 2014, Yuan-Sen Yang (Copyright holder)(mailto: yuansen@hotmail.com)  
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without modification, are permitted 
// provided that the following conditions are met: 
// 
// 1. Redistributions of source code must retain the above copyright notice, this list of 
//    conditions and the following disclaimer. 
// 2. Redistributions in binary form must reproduce the above copyright notice, this list of 
//    conditions and the following disclaimer in the documentation and/or other materials provided
//    with the distribution. 
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR
// IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY 
// AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
// OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE 
// POSSIBILITY OF SUCH DAMAGE.
// 
// The views and conclusions contained in the software and documentation are those of the authors
// and should not be interpreted as representing official policies, either expressed or implied, 
// of the FreeBSD Project.
//
// (FreeBSD License)

// 
// int pickAPoint(const std::string       & winName, 
//                         const cv::Mat           & userImage,             
//                         cv::Point2f             & pickedPoint, 
//                         int                       breakKey = 27) 
// 
// Description: 
//   pickAPoint() provides an interactive interface allowing user to pick a point from an
//     image. 
//     User instruction: 
//          Zoom-in the view:   Mouse-left-button-click
//          Zoom-out the view:  Ctrl mouse-left-button-click
//          Pan the view:       Press mouse-left-button and move before releasing
//          Pick point:         Mouse right-button-click. 
//
// Parameters: 
// 
//   const std::string       & winName
//     The window title of the interactive interface. 
// 
//   const cv::Mat           & userImage
//     User's image that will show in the interactive interface. 
//     
//   cv::Point2f             & pickedPoint
//     The output parameter of the picked point coordinate. 
//     The coordinate of the picked point is with respect to the user's image coordinate.
//     The coordinate is zero based.
//     The point (0.f,0.f) denotes the center point of the upper-left pixel square. That means  
//       the possible picked point range is X: (-0.5f, w - 1 + 0.5f), Y: (-0.5f, h - 1 + 0.5f), 
//       where w and h are the width and height of the user's image, respectively. 
// 
//   int                breakKey = 27
//     The break key that allows user to break this function. 
//     If user press breakKey before picking a function, this function stops and returns the 
//       breakKey value. If so, the parameter pickedPoint is unchanged. 
// 
// External data types and calls:
//     This function uses the following data types or classes:
//       cv::Mat (OpenCV)
//       cv::Rect (OpenCV)
//       cv::Point2f (OpenCV)
//       cv::Point2i (OpenCV) 
//       cv::Scalar (OpenCV)
//     This function calls the following external functions:
//       cv::resize (OpenCV)
//       cv::createWindow (OpenCV)
//       cv::imshow (OpenCV)
//       cv::waitKey (OpenCV)
//       cv::putText (OpenCV)
//       cv::setMouseCallback (OpenCV)
//       smoothZoomAndShow (smoothZoomAndShow.cpp) 
// 
// Developer: Yuan-Sen Yang
// 
// Date:  2014-01-28  Initial implementation 
//        2014-02-07  Renamed to pickAPoint
//        2014-02-25  Bug fixed (window name bug. added winname in PickPointSharedData)
//        2014-05-19  Bug fixed (window name bug. used winname when calling smoothZoomAndShow())
//        2014-05-19  Initialized thisData.show to Scalar(0,0,0)
//

#ifndef _pickAPoint_h_
#define _pickAPoint_h_

#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include "opencv2/highgui/highgui.hpp"

#include "smoothZoomAndShow.h"

using namespace cv;

int pickAPoint(
            const std::string & winName, 
            const cv::Mat & userImg, 
            cv::Point2f & pickedPoint, 
            int breakKey = 27, int confirmKey1 = 0, int confirmKey2 = 0); 

int pickATemplate(
	const std::string & winName,
	const cv::Mat & userImg,
	cv::Point2f & pickedPoint,
	cv::Rect & pickedRect,
	int breakKey = 27, int confirmKey1 = 0, int confirmKey2 = 0, 
	cv::Size initRoiSize = cv::Size(300, 300));

#endif 
