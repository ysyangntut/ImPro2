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
// int matchTemplateWithRotPyr(InputArray image, InputArray templ, 
//                                    double ref_x,    double ref_y, 
//                                    double min_x,    double max_x, 
//                                    double min_y,    double max_y, 
//                                    double min_rot,  double max_rot, 
//                                    vector<double> &  dispAndRot, 
//                                    int method); 
// 
// Description: 
//   matchTemplateWithRotPyr() runs template match considering ux, uy, and rotation 
//     by calling matchTemplateWithRot() many times in a multi-level (pyramid) way.
//
// Parameters: 
// 
//   InputArray              image 
//     Image where the search is running. (It must be 8-bit or 32-bit floating-point.?)
// 
//   InputArray              templ 
//     Searched template. It must be not greater than the source image and have the same data type.
//     
//   double                  ref_x, ref_y
//     reference point of the template (in pixel, upper-left is 0.0, 0.0) 
//
//   double                  min_x, max_x
//     searched range of x. If _min_x equals to _max_x, this dimension is not searched (is fixed)
//
//   double                  min_y, max_y
//     searched range of y. If _min_y equals to _max_y, this dimension is not searched (is fixed)
//
//   double                  min_rot, max_rot
//     searched range of rotation in degree. If _min_rot equals to _max_rot, this dimension is 
//     not searched (is fixed)
//
//   vector<double>        & result
//     searched most matched location and rotation (x, y, rotataion in degree)
//       result[0]:  px in pixel
//       result[1]:  py in pixel
//       result[2]:  rotation in degree
//       result[3]:  best matched value (method dependent)
//       result[4]:  total cpu time
//       result[5]:  cpu time on image resizing (cv::resize)
//       result[6]:  cpu time on image rotating (cv::getRotationMatrix2D and cv::warpAffine)
//       result[7]:  cpu time on template match (cv::matchTemplate)
//   
//   int                     return value
//      0: done successfully
//     -1: unsuccessfully
// 
//   

// 
// Developer: Yuan-Sen Yang
// 
// Date:  2014-05-12  initially implemented and tested
//        2014-05-19  bug fixed: accumulates timing statistics (result[4] ~ result[7])
//        2014-05-20  bug fixed: BUG: min_x   = max(min_x,   result[0] - 1.0 * precision_x); 
//                               FIX: min_x   = max(min_x,   result[0] - 1.0 * this_prec_x);  
//                               and so on.
//

#ifndef _matchTemplateWithRotPyr_
#define _matchTemplateWithRotPyr_

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <vector>

using namespace cv; 
using namespace std; 

int matchTemplateWithRotPyr(InputArray _image, InputArray _tmplt, 
                                   double ref_x,   double ref_y, 
                                   double min_x,   double max_x,   double precision_x, 
                                   double min_y,   double max_y,   double precision_y, 
                                   double min_rot, double max_rot, double precision_rot, 
                                   vector<double> &  result, 
                                   int method = cv::TM_CCORR_NORMED,
                                   double _init_prec_x = -1, double _init_prec_y = -1, double _init_prec_rot = -1); 

#endif 
