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
//  int smoothZoomAndShow(
//                        const cv::Mat  &   baseImage,            
//                        Rect &             rangeA,           
//                        Rect &             rangeB,           
//                        cv::Mat &          zoomMat,          
//                        cv::Mat &          showMat,          
//                        const string &     showWindowName,   
//                        int                numTransitionFrames, 
//                        int                waitKeyTime = 1,      
//                        int                transInterp = cv::INTER_NEAREST, 
//                        int                finalInterp = cv::INTER_AREA, 
//                        int                breakKey = 27 )
// 
// Description: 
//   smoothZoomAndShow() displays a smooth transition of zooming in or zooming out or moving
//   an image from range A to range B. 
//
// Parameters: 
// 
//   const cv::Mat  &   baseMat
//     The base image. 
// 
//   cv::Rect &   rangeA / rangeB
//     The beginning range (rangeA) and destination range (rangeB) of the transition. 
//     These are the first and the final ranges that will be displayed, respectively. 
//     The coordinates of rangeA and rangeB are with respect to baseMat. 
//     If the range is partially or entirely out of range of baseMat, it will be adjusted so 
//       that the range is entirely within the baseMat. This is why rangeA and rangeB are not
//       set to const because they will be changed if they are out of range. 
// 
//   cv::Mat &          zoomMat
//     The transition image. 
//     It is the image or memory space that is used to store the transition image. It should 
//       be a sub-matrix of showMat or the showMat itself so that the transition image can 
//       be displayed. 
//
//   cv::Mat &          showMat
//     The image that will be displayed in the display window during the transition. 
//     It should be the parent matrix of zoomMat or the same as zoomMat. If not, the transition 
//       image (zoomMat) will not be displayed, and you will not see the transition. 
//     The showMat is displayed by OpenCV's imshow() function.
//
//   const string &     showWindowName
//     The name of the display window that shows showMat. 
//     If the window is not created, a new window will be created. 
//
//   int                numTransitionFrames
//     Number of transition frames during the transition. 
//     It includes the beginning frame (rangeA) and the destination frame (rangeB). 
//     If numTransitionFrames is 1, only destination frame will show. 
//     numTransitionFrames must be >= 1. If not, this function returns -1.  
//
//   int                waitKeyTime
//     Number of milliseconds to wait between each frame of transition.
//     This is the parameter for OpenCV's waitKey(). waitKey() is called after each frame of 
//       transition is displayed. 
//
//   int                transInterp = cv::INTER_NEAREST
//     Interpolation method used when resizing a part of baseMat to zoomMat during the 
//       transition except the final frame. 
//     This is the parameter for OpenCV's resize(). resize() with parameter transInterp is called
//     (numZoomChange - 1) times during the transition. 
//     It should be one of the following value: INTER_NEAREST, INTER_LINEAR, INTER_AREA, 
//       INTER_CUBIC, INTER_LANCZOS4. 
//     The default value cv::INTER_NEAREST is selected because of its fast computing time. 
//
//   int                finalInterp = cv::INTER_AREA
//     Interpolation method used when resizing a final frame (rangeB). 
//     The final frame is displayed using this interpolation method. 
//     It should be one of the following value: INTER_NEAREST, INTER_LINEAR, INTER_AREA, 
//       INTER_CUBIC, INTER_LANCZOS4. 
//
//   int                breakKey = 27
//     The break key when running the transition
//     If user press breakKey during the transition, the transition stops and this function 
//       returns the breakKey value. 
//
// 
// External data types and calls:
//     This function uses the following data types or classes:
//       cv::Mat (OpenCV)
//       cv::Rect (OpenCV)
//     This function calls the following external functions:
//       cv::resize (OpenCV)
//       cv::imshow (OpenCV)
//       cv::waitKey (OpenCV)
// 
// Developer: Yuan-Sen Yang
// 
// Date:  2014-01-25  Initial implementation 
//        2014-02-07  Renamed to smoothZoomAndShow
//        2014-02-13  Bug fixed (bug: if aspect ratio is large, when zoom to limit size, the aspect
//                    ratio if shown image changes)
//

#ifndef _smoothZoomAndShow_h_
#define _smoothZoomAndShow_h_

#include <string>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

using namespace std; 

int smoothZoomAndShow(
            const cv::Mat  &   image,            
            cv::Rect &         rangeA,           
            cv::Rect &         rangeB,           
            cv::Mat &          zoomMat,          
            cv::Mat &          showMat,          
            const string &     showWindowName,   
            int                numTransitionFrames, 
            int                waitKeyTime,
            int                transInterp = cv::INTER_NEAREST,
            int                finalInterp = cv::INTER_AREA,
            int                breakKey = 27); 

#endif
