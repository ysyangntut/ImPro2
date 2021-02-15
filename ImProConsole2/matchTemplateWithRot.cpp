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
// Created: 2014-04-28 by Yuan-Sen Yang
// Modifed: 2015-08-02 by Yuan-Sen Yang (@ UIUC)
//            added openmp for rotation loop
//            used INTER_CUBIC for resize()
//            replaced resize() with remap() when scaling search image,
//              so that ranges do not need to be integer before
//              scaling, making ranges smaller and saving computin time.
//

#include "matchTemplateWithRot.h"
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <stdio.h>
#include <vector>
#include <cmath>
#include "impro_util.h"

using namespace cv; 
using namespace std; 

#define PI 3.14159265358979323846 

int matchTemplateWithRot(
	    InputArray search, InputArray tmplt, 
        double _ref_x,   double _ref_y, 
        double _min_x,   double _max_x,   double _precision_x, 
        double _min_y,   double _max_y,   double _precision_y, 
        double _min_rot, double _max_rot, double _precision_rot, 
        vector<double> &  result, 
        int      method)
{
  // Check
  cv::Mat tmpltMat  = tmplt.getMat(); 
  cv::Mat searchMat = search.getMat(); 
  if (tmpltMat.rows <= 0 || tmpltMat.cols <= 0 || 
     searchMat.rows <= 0 || searchMat.cols <= 0)
    return -1; 
//  if (tmpltMat.rows > searchMat.rows || tmpltMat.cols > searchMat.cols)
//    return -1;
  if (_ref_x < 0 || _ref_x > tmpltMat.cols - 1 ||
      _ref_y < 0 || _ref_y > tmpltMat.rows - 1)
    return -1;
  // timing
  double tTotal = 0.0, tResize = 0.0, tMatch = 0.0, tRotate = 0.0;
  double tStart1, tEnd1, tStart2, tEnd2; 
  tStart1 = getCpusTime(); 

  // Step 0:  Determine minReducedRatioDueToRotation, dDegree, nRot
  //          
  double minReducedRatioDueToRotation = 1.0; // 1.0 means no reduction
  double dDegree = _precision_rot + 1e-10; // initially set, will be slightly adjusted later.
  int    nRot;
  if (_min_rot != 0 || _max_rot != 0) { // check if rotation process is needed
      // make it between [-180,180)
      _max_rot = fmod(fmod(_max_rot + 180, 360) + 360, 360) - 180; 
	  // make it between [-180,180)
      _min_rot = fmod(fmod(_min_rot + 180, 360) + 360, 360) - 180; 
      nRot = std::max((int) ((_max_rot - _min_rot) / dDegree + 2), 2);
      dDegree = (_max_rot - _min_rot) * 1.0 / (nRot - 1);
      if (_min_rot == _max_rot)  nRot = 1;
        // run through all rotation angle and find the minimal reduced ratio
      for (int iRot = 0; iRot < nRot; iRot++) {
        double reducedRatio, th;
        th = (_min_rot + iRot * dDegree) * PI / 180.0;
        th = acos(cos(4 * th)) / 4; // equivalent angle that has equal rotational 
		                            // boundary cutting ratio
        reducedRatio = 1. / (cos(th) + sin(th)); // or sin(0.25*PI) / (sin(0.25*PI+th); 
		                                         // both are the same
        if (reducedRatio < minReducedRatioDueToRotation)
          minReducedRatioDueToRotation = reducedRatio;
      }
  } else {
      minReducedRatioDueToRotation = 1.0;  // template size does not need to be reduced.
      dDegree = 0.0;  // whatever
      nRot = 1;
  }
  //{
  //  // debug
  //  cv::Mat imgToFind = tmpltMat; 
  //  cv::Mat tmpltCorner(1, 1, CV_32FC2); 
  //  tmpltCorner.at<cv::Point2f>(0, 0).x = 100;
  //  tmpltCorner.at<cv::Point2f>(0, 0).y = 100; 
  //  cv::Mat tmpltBW; cv::cvtColor(imgToFind, tmpltBW, CV_BGR2GRAY);
  //  cv::cornerSubPix(tmpltBW, tmpltCorner, cv::Size(10, 10), 
  //    cv::Size(-1,-1), 
  //    cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 100, 0.01));
  //  double cx = tmpltCorner.at<cv::Point2f>(0, 0).x; 
  //  double cy = tmpltCorner.at<cv::Point2f>(0, 0).y; 
  //}

  // Step 1:  Determine the template.
  //          If rotation is needed, make the template square
  int tx0, ty0, tx1, ty1; // template range (width is tx1 - tx0)
  if (_min_rot != 0.0 || _max_rot != 0.0) {
      tx0 = (int) (std::max(_ref_x - _ref_y, 0.0) + 0.5);
      ty0 = (int) (std::max(_ref_y - _ref_x, 0.0) + 0.5);
      tx1 = (int) (std::min(tx0 + 2 * min(_ref_x, _ref_y) + 1, 1.0 * tmpltMat.cols));
      ty1 = (int) (std::min(ty0 + 2 * min(_ref_x, _ref_y) + 1, 1.0 * tmpltMat.rows));
  } else {
      // Rotation is not needed. Template does not need to be square.
      tx0 = 0;
      ty0 = 0;
      tx1 = tmpltMat.cols;
      ty1 = tmpltMat.rows;
  }

  // Step 2:  Reduce the square template for rotation without going beyond boundary,
  //          scaleFactorX, scaleFactorY
  int txr0, txr1, tyr0, tyr1;
  if (_min_rot != 0. || _max_rot != 0.) {
      txr0 = tx0 + (int) ((tx1 - tx0) * (1. - minReducedRatioDueToRotation) / 2. + .99);
      txr1 = tx1 - (int) ((tx1 - tx0) * (1. - minReducedRatioDueToRotation) / 2. + .99);
      tyr0 = ty0 + (int) ((ty1 - ty0) * (1. - minReducedRatioDueToRotation) / 2. + .99);
      tyr1 = ty1 - (int) ((ty1 - ty0) * (1. - minReducedRatioDueToRotation) / 2. + .99);
  } else {
      txr0 = tx0;
      txr1 = tx1;
      tyr0 = ty0;
      tyr1 = ty1;
  }

  //    Check if template (txr0~r1, tyr0~ry) is larger that search
  //    (modified by vince 2015-08-25)

  if ((txr1 - txr0) > searchMat.cols) {
      // put ref_x at center of min_x max_x in search image, and check where the 
	  // boundary of template is
      double b_lo = 0.5 * (_min_x + _max_x) - (_ref_x - txr0);
      double b_hi = b_lo + (txr1 - txr0);
      if (b_lo < 0 && b_hi <= searchMat.cols) {
          // case 1: trim left or upper part of template
          txr0 = txr1 - searchMat.cols;
      } else if (b_lo < 0 && b_hi > searchMat.cols) {
          // case 2: trim both sides of templates
          txr0 -= (int) b_lo;
          txr1 = txr0 + searchMat.cols;
      } else {
          // case 3: trim right or lower part of template
          txr1 = txr0 + searchMat.cols;
      }
  } // end of if reduced template is still wider than search width

  if ((tyr1 - tyr0) > searchMat.rows) {
      // put ref_y at center of min_y max_y in search image, and check where the 
	  // boundary of template is
      double b_lo = 0.5 * (_min_y + _max_y) - (_ref_y - tyr0);
      double b_hi = b_lo + (tyr1 - tyr0);
      if (b_lo < 0 && b_hi <= searchMat.rows) {
          // case 1: trim left or upper part of template
          tyr0 = tyr1 - searchMat.rows;
      } else if (b_lo < 0 && b_hi > searchMat.rows) {
          // case 2: trim both sides of templates
          tyr0 -= (int) b_lo;
          tyr1 = tyr0 + searchMat.rows;
      } else {
          // case 3: trim right or lower part of template
          tyr1 = tyr0 + searchMat.rows;
      }
  } // end of if reduced template is still higher than search height

  //   Ensure the template is square (if the rotation is needed)
  if (_min_rot != 0 || _max_rot != 0) {
      int diff = (tyr1 - tyr0) - (txr1 - txr0);
      if (diff > 0) {
          // if height > width
          tyr0 += diff / 2;
          tyr1 = tyr0 + (txr1 - txr0);
      }
      if (diff < 0) {
 //         txr0 += diff / 2;  // bug found by vince 2018-08-10
          txr0 -= diff / 2;    // bug found by vince 2018-08-10
		  txr1 = txr0 + (tyr1 - tyr0);
      }
  }

  // Step 3:  Determine scaled template size scaledTmpltSizeX, scaledTmpltSizeY
  
  //          squareTmplt: the square template, a sub-image of tmplt
  //          squareTmpltRotated: to be rotated image of square template, 
  //                              which has black area
  //          squareTmpltRotatedReduced: reduced rotated image of square template, 
  //                              no black area
  //          squareTmpltScaled: scaled image of squareTmpltRotatedReduced

  double scaleFactorX = 1. / _precision_x; // Note: This factor will be updated soon.
  double scaleFactorY = 1. / _precision_y; // Note: This factor will be updated soon.
  int scaledTmpltSizeX = (int) (scaleFactorX * (txr1 - txr0 -1) + 1.99);
  int scaledTmpltSizeY = (int) (scaleFactorY * (tyr1 - tyr0 -1) + 1.99);
  //  int scaledTmpltSizeX = (int) ( (txr1 - txr0) / _precision_x + 0.99);
  //  int scaledTmpltSizeY = (int) ( (tyr1 - tyr0) / _precision_y + 0.99);
  cv::Size scaledSize(scaledTmpltSizeX, scaledTmpltSizeY);
  scaleFactorX = (scaledTmpltSizeX - 1) * 1.0 / (txr1 - txr0 - 1);
  scaleFactorY = (scaledTmpltSizeY - 1) * 1.0 / (tyr1 - tyr0 - 1);
  //  scaleFactorX = scaledTmpltSizeX * 1.0 / (txr1 - txr0);
  //  scaleFactorY = scaledTmpltSizeY * 1.0 / (tyr1 - tyr0);
  // Image size concept: If you resize an image from width W pixels to N * W pixels,
  //                     object width in the image is NOT from 1 to N, but from
  //                     1 to (NW - 1) / (W - 1). Think again and try to understand.

  cv::Mat squareTmplt = tmpltMat(cv::Rect(tx0, ty0, tx1 - tx0, ty1 - ty0)); 
                        // square template, a sub-image of template
//  cv::Mat squareTmpltRotated;               // rotated square template
//  cv::Mat squareTmpltRotatedCropped;        // rotated and cropped template
//  cv::Mat squareTmpltRotatedCroppedScaled;  // scaled squareTmpltRotatedCropped
//  cv::Mat searchCropped;                    // cropped search image
//  cv::Mat searchCroppedScaled;              // cropped and scaled search image
  cv::Mat searchResampled;                  // resampled search image 
                                            // (scaling + cropping in one step)

  //{
  //  // debug
  //  cv::Mat imgToFind = squareTmplt; 
  //  cv::Mat tmpltCorner(1, 1, CV_32FC2); 
  //  tmpltCorner.at<cv::Point2f>(0, 0).x = 100;
  //  tmpltCorner.at<cv::Point2f>(0, 0).y = 100; 
  //  cv::Mat tmpltBW; cv::cvtColor(imgToFind, tmpltBW, CV_BGR2GRAY);
  //  cv::cornerSubPix(tmpltBW, tmpltCorner, cv::Size(10, 10), 
  //    cv::Size(-1,-1), 
  //    cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 100, 0.01));
  //  double cx = tmpltCorner.at<cv::Point2f>(0, 0).x; 
  //  double cy = tmpltCorner.at<cv::Point2f>(0, 0).y; 
  //}

  // Step 4:      scale and crop search to a smaller size for speed

  // calculate search range (do not need to be integers)
  double searchRect_x0 = _min_x - _ref_x + txr0; // new version searchRect_x does not need 
  double searchRect_y0 = _min_y - _ref_y + tyr0; // to be integer. (2015-08-03)
  if (searchRect_x0 < 0.) searchRect_x0 = 0.;
  if (searchRect_y0 < 0.) searchRect_y0 = 0.;
  double searchRect_x1 = _max_x - _ref_x + txr1 - 0.0; // right bound of search range
  double searchRect_y1 = _max_y - _ref_y + tyr1 - 0.0; // bottom bound of search range
  // calculate scaled search range image size (need to be integers)
  int scaledSearchImgWidth = (int) ((searchRect_x1 - searchRect_x0) * scaleFactorX + 2);
  int scaledSearchImgHeight = (int) ((searchRect_y1 - searchRect_y0) * scaleFactorY + 2);
  // search image has to be >= template image (// 2018-08-10 vince)
  if (scaledSearchImgWidth < scaledSize.width) scaledSearchImgWidth = scaledSize.width; 
  if (scaledSearchImgHeight < scaledSize.height) scaledSearchImgHeight = scaledSize.height; 
																						// generate map coordinates
  tStart2 = getCpusTime();
  cv::Mat mapx(scaledSearchImgHeight, scaledSearchImgWidth, CV_32FC1);
  cv::Mat mapy(scaledSearchImgHeight, scaledSearchImgWidth, CV_32FC1);
  double _y, dx, dy;
  dx = 1.0 / scaleFactorX;
  dy = 1.0 / scaleFactorY;
  for (int i = 0; i < scaledSearchImgHeight; i++) {
      _y = searchRect_y0 + i * dy;
      for (int j = 0; j < scaledSearchImgWidth; j++) {
          mapx.at<float>(i,j) = (float) (searchRect_x0 + j * dx);
          mapy.at<float>(i,j) = (float) (_y);
      }
  }
  cv::remap(searchMat, searchResampled, mapx, mapy, cv::INTER_CUBIC);
  tEnd2 =   getCpusTime(); tResize += tEnd2 - tStart2; 

  // Step 5:  For each rotation
    // run through all rotation angle
  double best_matched_value;
  double best_matched_theta; 
  double best_matched_x; 
  double best_matched_y; 
//#pragma omp parallel for private(tStart2, tEnd2) shared(tRotate, tResize, tMatch)
  for (int iRot = 0; iRot < nRot; iRot++) {
    cv::Mat squareTmpltRotated;               // rotated square template
    cv::Mat squareTmpltRotatedCropped;        // rotated and cropped template
    cv::Mat squareTmpltRotatedCroppedScaled;  // scaled squareTmpltRotatedCropped

    double thdeg;
    thdeg = _min_rot + iRot * dDegree; 

    // Step 6:      generate rotated template (squareTmpltRotated)
    tStart2 = getCpusTime(); 
    cv::Point2f rotationCenter((float) _ref_x - tx0, (float) _ref_y - ty0); 
    cv::Mat r = cv::getRotationMatrix2D(rotationCenter, thdeg, 1.0);
    cv::warpAffine(squareTmplt, squareTmpltRotated, r, cv::Size(tx1 - tx0, ty1 - ty0),
                   CV_INTER_CUBIC);
    tEnd2   = getCpusTime(); tRotate += tEnd2 - tStart2; 

    //{
    //  // debug
    //  cv::Mat imgToFind = squareTmpltRotated; 
    //  cv::Mat tmpltCorner(1, 1, CV_32FC2); 
    //  tmpltCorner.at<cv::Point2f>(0, 0).x = 100;
    //  tmpltCorner.at<cv::Point2f>(0, 0).y = 100; 
    //  cv::Mat tmpltBW; cv::cvtColor(imgToFind, tmpltBW, CV_BGR2GRAY);
    //  cv::cornerSubPix(tmpltBW, tmpltCorner, cv::Size(10, 10), 
    //    cv::Size(-1,-1), 
	//    cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 100, 0.01));
    //  double cx = tmpltCorner.at<cv::Point2f>(0, 0).x; 
    //  double cy = tmpltCorner.at<cv::Point2f>(0, 0).y; 

    //  double s = 5.5555555555556; 
    //  cv::Mat sImg; 
    //  cv::resize(imgToFind, sImg, cv::Size(0,0), s, s); 
    //  cx = tmpltCorner.at<cv::Point2f>(0, 0).x = (cx + 0.5) * s - 0.5;
    //  cy = tmpltCorner.at<cv::Point2f>(0, 0).y = (cy + 0.5) * s - 0.5; 
    //  tmpltBW; cv::cvtColor(sImg, tmpltBW, CV_BGR2GRAY);
    //  cv::cornerSubPix(tmpltBW, tmpltCorner, cv::Size(10, 10), 
    //    cv::Size(-1,-1), 
	//    cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 100, 0.01));
    //  cx = tmpltCorner.at<cv::Point2f>(0, 0).x; 
    //  cy = tmpltCorner.at<cv::Point2f>(0, 0).y; 
    //}

    // Step 7:      crop rotated template  (squareTmpltRotatedCropped)
    squareTmpltRotatedCropped = squareTmpltRotated(cv::Rect(txr0 - tx0, tyr0 - ty0, 
	                                                        txr1 - txr0, tyr1 - tyr0)); 
  //cv::imshow("squareTmplt", squareTmplt); cv::waitKey(); 
  //cv::imshow("squareTmpltRotated", squareTmpltRotated); cv::waitKey(); 
  //cv::imshow("squareTmpltRotatedCropped", squareTmpltRotatedCropped); cv::waitKey(); 

    //{
    //  // debug
    //  cv::Mat imgToFind = squareTmpltRotatedCropped; 
    //  cv::Mat tmpltCorner(1, 1, CV_32FC2); 
    //  tmpltCorner.at<cv::Point2f>(0, 0).x = 90;
    //  tmpltCorner.at<cv::Point2f>(0, 0).y = 90; 
    //  cv::Mat tmpltBW; cv::cvtColor(imgToFind, tmpltBW, CV_BGR2GRAY);
    //  cv::cornerSubPix(tmpltBW, tmpltCorner, cv::Size(10, 10), 
    //    cv::Size(-1,-1), 
	//    cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 100, 0.01));
    //  double cx = tmpltCorner.at<cv::Point2f>(0, 0).x; 
    //  double cy = tmpltCorner.at<cv::Point2f>(0, 0).y; 
    //}

    // Step 8:      scale squareTmpltRotatedCropped to a smaller size for speed
    tStart2 = getCpusTime(); 
    cv::resize(squareTmpltRotatedCropped, squareTmpltRotatedCroppedScaled,
               scaledSize, 0, 0, cv::INTER_LANCZOS4);
    tEnd2 =   getCpusTime(); tResize += tEnd2 - tStart2; 

    //{
    //  // debug
    //  cv::Mat imgToFind = squareTmpltRotatedCroppedScaled; 
    //  cv::Mat tmpltCorner(1, 1, CV_32FC2); 
    //  tmpltCorner.at<cv::Point2f>(0, 0).x = (89.5 + 0.5) * scaleFactorX - 0.6;
    //  tmpltCorner.at<cv::Point2f>(0, 0).y = (89.5 + 0.5) * scaleFactorX - 0.6;
    //  cv::Mat tmpltBW; cv::cvtColor(imgToFind, tmpltBW, CV_BGR2GRAY);
    //  cv::cornerSubPix(tmpltBW, tmpltCorner, cv::Size(10, 10), 
    //    cv::Size(-1,-1), 
	//    cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 100, 0.01));
    //  double cx = tmpltCorner.at<cv::Point2f>(0, 0).x; 
    //  double cy = tmpltCorner.at<cv::Point2f>(0, 0).y; 
    //}
    //{
    //  // debug
    //  cv::Mat imgToFind = searchMat; 
    //  cv::Mat tmpltCorner(1, 1, CV_32FC2); 
    //  tmpltCorner.at<cv::Point2f>(0, 0).x = 135.5 + 0.12345;
    //  tmpltCorner.at<cv::Point2f>(0, 0).y = 135.5 + 0.12345; 
    //  cv::Mat tmpltBW; cv::cvtColor(imgToFind, tmpltBW, CV_BGR2GRAY);
    //  cv::cornerSubPix(tmpltBW, tmpltCorner, cv::Size(10, 10), 
    //    cv::Size(-1,-1), 
	//    cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 100, 0.01));
    //  double cx = tmpltCorner.at<cv::Point2f>(0, 0).x; 
    //  double cy = tmpltCorner.at<cv::Point2f>(0, 0).y; 
    //}
    //{
    //  // debug
    //  cv::Mat imgToFind = searchScaled; 
    //  cv::Mat tmpltCorner(1, 1, CV_32FC2); 
    //  tmpltCorner.at<cv::Point2f>(0, 0).x = (135.5 + 0.5) * scaleFactorX - 0.5;
    //  tmpltCorner.at<cv::Point2f>(0, 0).y = (135.5 + 0.5) * scaleFactorY - 0.5;
    //  cv::Mat tmpltBW; cv::cvtColor(imgToFind, tmpltBW, CV_BGR2GRAY);
    //  cv::cornerSubPix(tmpltBW, tmpltCorner, cv::Size(10, 10), 
    //    cv::Size(-1,-1), 
	//    cv::TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 100, 0.01));
    //  double cx = tmpltCorner.at<cv::Point2f>(0, 0).x; 
    //  double cy = tmpltCorner.at<cv::Point2f>(0, 0).y; 
    //}

    // Step 9:      run template match
    double scaledMinVal, scaledMaxVal;
    cv::Point scaledMinLoc, scaledMaxLoc;
    cv::Point2f minLoc, maxLoc; 
    cv::Mat matchResult;
    tStart2 = getCpusTime();
//    cv::imshow("searchResampled", searchResampled); cv::waitKey(-1);
//    cv::imshow("template", squareTmpltRotatedCroppedScaled); cv::waitKey(-1);
    cv::matchTemplate(searchResampled, squareTmpltRotatedCroppedScaled, 
	                  matchResult, method);
    //cv::imshow("searchScaled", searchScaled); 
    //cv::imshow("squareTmpltRotatedCroppedScaled", squareTmpltRotatedCroppedScaled); 
    //cv::imshow("matchResult", matchResult); cv::waitKey(); 
    cv::minMaxLoc( matchResult, &scaledMinVal, &scaledMaxVal, 
	               &scaledMinLoc, &scaledMaxLoc, Mat() );
	// added by vince on 10 Dec 2018 for subpixel estimation
	cv::Point2f scaledMaxLocFloat((float)scaledMaxLoc.x, (float)scaledMaxLoc.y); // for subpixel estimation
	float v11 = matchResult.at<float>(scaledMaxLoc.y, scaledMaxLoc.x);
	if (scaledMaxLoc.x > 0 && scaledMaxLoc.x < matchResult.cols - 1) {
		float v10 = matchResult.at<float>(scaledMaxLoc.y, scaledMaxLoc.x - 1);
		float v12 = matchResult.at<float>(scaledMaxLoc.y, scaledMaxLoc.x + 1);
		float dYL = v11 - v10, dYR = v11 - v12; 
		scaledMaxLocFloat.x += 0.5f * (dYL - dYR) / (dYL + dYR); 
	}
	if (scaledMaxLoc.y > 0 && scaledMaxLoc.y < matchResult.rows - 1) {
		float v01 = matchResult.at<float>(scaledMaxLoc.y - 1, scaledMaxLoc.x);
		float v21 = matchResult.at<float>(scaledMaxLoc.y + 1, scaledMaxLoc.x);
		float dYU = v11 - v01, dYD = v11 - v21;
		scaledMaxLocFloat.y += 0.5f * (dYU - dYD) / (dYU + dYD);
	}
	tEnd2   = getCpusTime(); tMatch += tEnd2 - tStart2;
//    printf("S-sz:%dx%d, T-sz:%dx%d, CPU:%f\n",
//      searchResampled.rows, searchResampled.cols,
//      squareTmpltRotatedCroppedScaled.rows, squareTmpltRotatedCroppedScaled.cols,
//      tEnd2 - tStart2);
    //maxLoc.x = (scaledMaxLoc.x + 0.5f) / scaleFactorX - 0.5f;
    //maxLoc.y = (scaledMaxLoc.y + 0.5f) / scaleFactorY - 0.5f; 
//    maxLoc.x = (float) (scaledMaxLoc.x / scaleFactorX);
//    maxLoc.y = (float) (scaledMaxLoc.y / scaleFactorY); 
    maxLoc.x = (float) (scaledMaxLocFloat.x / scaleFactorX); // modified by vince on 10 Dec 2018 for subpixel estimation
    maxLoc.y = (float) (scaledMaxLocFloat.y / scaleFactorY); // modified by vince on 10 Dec 2018 for subpixel estimation
	maxLoc.x += (float) ((_ref_x - txr0) + searchRect_x0);
    maxLoc.y += (float) ((_ref_y - tyr0) + searchRect_y0);

    // Step 10:      update best result
    if (iRot == 0 || scaledMaxVal > best_matched_value) {
      best_matched_value = scaledMaxVal; 
      best_matched_theta = thdeg; 
      best_matched_x     = maxLoc.x; 
      best_matched_y     = maxLoc.y; 
    }
    if (_min_x == _max_x) {
        best_matched_x = _min_x;
    }
    if (_min_y == _max_y) {
        best_matched_y = _min_y;
    }

    // debug: print result for debug
//    printf("Rot:%9.4f  Ux:%9.4f  Uy:%9.4f  Corr:%9.6f\n", 
//                       thdeg, maxLoc.x, maxLoc.y, scaledMaxVal); 

  }

  // debug: print result for debug
//  printf("Best match Rot:%9.4f  Ux:%9.4f  Uy:%9.4f  Corr:%9.6f\n\n", 
//          best_matched_theta, best_matched_x, best_matched_y, best_matched_value); 

  // Step 10:  return result
    // timing
  tEnd1 = getCpusTime();
  tTotal = tEnd1 - tStart1;
  result.resize(8);
  result[0] = best_matched_x;
  result[1] = best_matched_y;
  result[2] = best_matched_theta;
  result[3] = best_matched_value; 
  result[4] = tTotal; 
  result[5] = tResize;
  result[6] = tRotate;
  result[7] = tMatch; 

  return 0; 
}
