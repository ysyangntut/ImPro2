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



#include <string>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "impro_util.h"

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
            int                breakKey = 27) 
{
  // check image size
  if (image.rows <= 0 || image.cols <= 0) return -1; 
  int x_rangeA, y_rangeA, w_rangeA, h_rangeA;
  int x_rangeB, y_rangeB, w_rangeB, h_rangeB;
  // copy data from rangeA/B 
  // (as they can be changed if they are out of range)
  x_rangeA = rangeA.x; 
  y_rangeA = rangeA.y;
  w_rangeA = rangeA.width;
  h_rangeA = rangeA.height;
  x_rangeB = rangeB.x; 
  y_rangeB = rangeB.y;
  w_rangeB = rangeB.width;
  h_rangeB = rangeB.height;
  // check size of rangeA/B (should be >= 6-by-6)  // modified by vince: 2014-02-13 
  if (w_rangeA < 6) { 
    // keep rangeA aspect ratio unchanged
    h_rangeA = (int) ((h_rangeA * 6) / w_rangeA + 0.5); // modified by vince: 2014-02-13 
    w_rangeA = 6;
  }
  if (h_rangeA < 6) {
    // keep rangeA aspect ratio unchanged
    w_rangeA = (int) ((w_rangeA * 6) / h_rangeA + 0.5); // modified by vince: 2014-02-13 
    h_rangeA = 6;
  }
  if (w_rangeB < 6) { 
    // keep rangeB aspect ratio unchanged
    h_rangeB = (int) ((h_rangeB * 6) / w_rangeB + 0.5); // modified by vince: 2014-02-13 
    w_rangeB = 6;
  }
  if (h_rangeB < 6) {
    // keep rangeB aspect ratio unchanged
    w_rangeB = (int) ((w_rangeB * 6) / h_rangeB + 0.5); // modified by vince: 2014-02-13 
    h_rangeB = 6;
  }
  if (w_rangeA > image.cols) {
    w_rangeA = image.cols; 
    x_rangeA = 0;
  }
  if (h_rangeA > image.rows) {
    h_rangeA = image.rows; 
    y_rangeA = 0;
  }
  if (w_rangeB > image.cols) {
    w_rangeB = image.cols; 
    x_rangeB = 0;
  }
  if (h_rangeB > image.rows) {
    h_rangeB = image.rows; 
    y_rangeB = 0;
  }
  //  check positions of rangeA/B
  if (x_rangeA < 0) x_rangeA = 0;
  if (y_rangeA < 0) y_rangeA = 0;
  if (x_rangeA + w_rangeA > image.cols) x_rangeA = image.cols - w_rangeA; 
  if (y_rangeA + h_rangeA > image.rows) y_rangeA = image.rows - h_rangeA; 
  if (x_rangeB < 0) x_rangeB = 0;
  if (y_rangeB < 0) y_rangeB = 0;
  if (x_rangeB + w_rangeB > image.cols) x_rangeB = image.cols - w_rangeB; 
  if (y_rangeB + h_rangeB > image.rows) y_rangeB = image.rows - h_rangeB; 
  // (update rangeA and rangeB as they might be changed if they were out of range)
  rangeA.x      =   x_rangeA; 
  rangeA.y      =   y_rangeA; 
  rangeA.width  =   w_rangeA; 
  rangeA.height =   h_rangeA; 
  rangeB.x      =   x_rangeB; 
  rangeB.y      =   y_rangeB; 
  rangeB.width  =   w_rangeB; 
  rangeB.height =   h_rangeB; 
  // check zoomMat 
  if (zoomMat.rows <= 0 || zoomMat.cols <= 0) return -1; 
  // check showMat 
  if (showMat.rows <= 0 || showMat.cols <= 0) return -1; 
  // check numTransitionFrames
  if (numTransitionFrames < 1) return -1; 
  // check waitKeyTime
  if (waitKeyTime < 1) waitKeyTime = 1; 
  // smooth zoom
  int i, interpMethod; 
  float transition;  // from 0.0f to 1.0f 
  cv::Rect rangeT;   // transition range between rangeA and rangeB
  for (i = 0; i < numTransitionFrames; i++) {
    // transition is from 0 to 1 through the loop
    if (numTransitionFrames == 1) 
      transition = 1.f;
    else 
      transition = (1.f - cos(i * 3.141592653f / (numTransitionFrames - 1))) / 2.f; 
    rangeT.x      = (int) (transition * x_rangeB + (1.f - transition) * x_rangeA + 0.5f); 
    rangeT.y      = (int) (transition * y_rangeB + (1.f - transition) * y_rangeA + 0.5f); 
    rangeT.width  = (int) (transition * w_rangeB + (1.f - transition) * w_rangeA + 0.5f); 
    rangeT.height = (int) (transition * h_rangeB + (1.f - transition) * h_rangeA + 0.5f); 
    // resize 
    if (i < numTransitionFrames - 1) 
      interpMethod = transInterp; 
    else 
      interpMethod = finalInterp; 
    cv::resize(image(rangeT), 
               zoomMat, 
               cv::Size(zoomMat.cols, zoomMat.rows), 0, 0, interpMethod); 
    // show updated showMat (the parent image of zoomMat)
    cv::imshow(showWindowName, showMat); 
    if (cv::waitKey(waitKeyTime) == breakKey) break; 
  }
  if (i < numTransitionFrames) return breakKey;
  return 0;
}
