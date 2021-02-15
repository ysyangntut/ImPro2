// Copyright (c) 2015, Yuan-Sen Yang (Copyright holder)(mailto: yuansen@hotmail.com)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice, this
//    list of conditions and the following disclaimer.
// 2. Redistributions in binary form must reproduce the above copyright notice, this
//    list of conditions and the following disclaimer in the documentation and/or
//    other materials provided with the distribution.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
// OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
// SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
// TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
// BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH
// DAMAGE.
//
// The views and conclusions contained in the software and documentation are those of
// the authors and should not be interpreted as representing official policies, either
// expressed or implied, of the FreeBSD Project.
//
// (FreeBSD License)


// 
// Developer: Yuan-Sen Yang

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "matchTemplateWithRot.h"
#include "matchTemplateWithRotPyr.h"

int matchTemplateWithRotPyr(
       InputArray _image, InputArray _tmplt, 
       double _ref_x,   double _ref_y, 
       double _min_x,   double _max_x,   double _precision_x, 
       double _min_y,   double _max_y,   double _precision_y, 
       double _min_rot, double _max_rot, double _precision_rot, 
       vector<double> &  result, 
       int method, 
       double _init_prec_x, double _init_prec_y, double _init_prec_rot)
{
  cv::Mat search = _image.getMat(); 
  cv::Mat tmplt  = _tmplt.getMat(); 
  double min_x, max_x, min_y, max_y, min_rot, max_rot, ref_x, ref_y; 
  double precision_x, precision_y, precision_rot; 
  double this_prec_x, this_prec_y, this_prec_rot; 

  if (tmplt.rows <= 0  || tmplt.cols <= 0 ||
      search.rows <= 0 || search.cols <= 0)
    return -1;

  // Check data
  //   Check search size, tmplt size
  //   Check parameter values
  min_x   = _min_x; 
  max_x   = max(min_x,   _max_x  ); 
  min_y   = _min_y; 
  max_y   = max(min_y,   _max_y  ); 
  min_rot = _min_rot; 
  max_rot = max(min_rot, _max_rot);
  if (max_rot - min_rot < 1e-3) max_rot = min_rot; // avoid floating-point error
  ref_x   = _ref_x; 
  ref_y   = _ref_y; 

  // default values
  if (_precision_x > 0) 
    precision_x = _precision_x; 
  else 
    precision_x = 1.0; 
  if (_precision_y > 0) 
    precision_y = _precision_y; 
  else 
    precision_x = 1.0; 
  if (_precision_rot > 0) 
    precision_rot = _precision_rot; 
  else 
    precision_rot = min(precision_x, precision_y) * 2.0 
                  * 180.0 / 3.1416 / min(tmplt.rows, tmplt.cols); 
  if (_init_prec_x > 0) 
    this_prec_x = _init_prec_x; 
  else
    this_prec_x = tmplt.cols / 32. ; 
  if (_init_prec_y > 0) 
    this_prec_y = _init_prec_y; 
  else
    this_prec_y = tmplt.rows / 32.; 
  if (_init_prec_rot > 0) 
    this_prec_rot = _init_prec_rot; 
  else 
    this_prec_rot = min(this_prec_x, this_prec_y) * 2.0 
                  * 180.0 / 3.1416 / min(tmplt.rows, tmplt.cols); 

//  printf("Pyramid initial prec_x/prec_y/prec_rot: %9.2f %9.2f %9.2f\n", 
//                  this_prec_x, this_prec_y, this_prec_rot); 

  // timing data for accumulation
  double timing[] = {0, 0, 0, 0}; 
  while (true) {
    //printf("matching x:%7.3f~%7.3f(%7.3f) y:%7.3f~%7.3f(%7.3f) rot:%7.3f~%7.3f(%7.3f)...\n", 
    //        min_x,   max_x,   this_prec_x, 
    //        min_y,   max_y,   this_prec_y,
    //        min_rot, max_rot, this_prec_rot); 
    matchTemplateWithRot(search, tmplt, 
                                   ref_x, ref_y,
                                   min_x,   max_x,   this_prec_x, 
                                   min_y,   max_y,   this_prec_y,
                                   min_rot, max_rot, this_prec_rot, 
                                   result, CV_TM_CCORR_NORMED); 
    // accumulating timing data.
    timing[0] += result[4]; 
    timing[1] += result[5]; 
    timing[2] += result[6]; 
    timing[3] += result[7]; 
    // printf("match x:%7.3f y:%7.3f rot:%7.3f value:%9.7f\n",
    //         result[0], result[1], result[2], result[3]);
    // printf("CPU Time: Total:%9.3f Resize:%9.3f Rotate:%9.3f  Match%9.3f\n",
    //   result[4], result[5], result[6], result[7]);

    min_x   = max(min_x,   result[0] - 3.0 * this_prec_x);
    max_x   = min(max_x,   result[0] + 3.0 * this_prec_x); 
    max_x   = max(min_x,   max_x); 
    min_y   = max(min_y,   result[1] - 3.0 * this_prec_y); 
    max_y   = min(max_y,   result[1] + 3.0 * this_prec_y); 
    max_y   = max(min_y,   max_y); 
    min_rot = max(min_rot, result[2] - 2.0 * this_prec_rot); 
    max_rot = min(max_rot, result[2] + 2.0 * this_prec_rot); 
    max_rot = max(min_rot, max_rot); 
    if (this_prec_x <= precision_x && this_prec_y <= precision_y 
	 && this_prec_rot <= precision_rot )
      break;
    if (this_prec_x   > precision_x  ) this_prec_x   *= 0.5; 
    if (this_prec_y   > precision_y  ) this_prec_y   *= 0.5; 
    if (this_prec_rot > precision_rot) this_prec_rot *= 0.5; 
  }
  // accumulated timing data.
  result[4] = timing[0]; 
  result[5] = timing[1]; 
  result[6] = timing[2]; 
  result[7] = timing[3]; 
  return 0; 
}
