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
// Developer: Yuan-Sen Yang
// Created: 2014-02-07 by Yuan-Sen Yang
// Modifed: 2015-08-04 by Yuan-Sen Yang (@ UIUC->TPE)
//          for opencv 3 (as mouse flag definition changes from 2.x to 3.0)
//


#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#include "opencv2/core/core_c.h"
#include "opencv2/imgproc/imgproc_c.h"
#include "opencv2/highgui/highgui_c.h"

#include "smoothZoomAndShow.h"
#include "pickAPoint.h"

#include "impro_util.h"

using namespace cv;

#ifdef _WIN32
#define SPRINTF sprintf_s 
#else
#define SPRINTF sprintf
#endif

#define MAX_IMAGE_WIN_WIDTH  500
#define MAX_IMAGE_WIN_HEIGHT 500

// external functions
//double getWallTime(); // get wall time
//double getCpusTime(); // get cpus time (sum of cpus usage. if multi-threading/multi-core, 
//                      // cpus time may be >= wall time)

// callback functions 
void mouseEvent(int event, int x, int y, int flags, void *param); 

// internal functions (some are called by callback functions)
int  printUserImageInfo(struct PickPointSharedData &); 
int  printInstruction(struct PickPointSharedData &);
int  updateMouseCoord(int event, int x, int y, int flag, struct PickPointSharedData &);
int  zoomAndUpdateZoominRange(struct PickPointSharedData &, float furtherZoomFact = 2.0f); 
int  drawZoominRangeInZoomout(struct PickPointSharedData &); 
int  PickPointSharedData_Init(struct PickPointSharedData &); 

// Define an internal struct that contains most of the window/image/position data
struct PickPointSharedData {
  // user image data
  cv::Mat uimg;                 // user image
  int h_uimg, w_uimg;       // Size of user image
  // zoomin image
  cv::Mat zoomin; 
  int w_zoomin, h_zoomin;   // size of zoomin image
  int x_zoomin, y_zoomin;   // position of zoomin image
  cv::Rect zoominRange;     // zoomin range wrt user image
  // zoomout image   
  cv::Mat zoomout; 
  int w_zoomout, h_zoomout; // size of zoomout image (resized user image)
  int x_zoomout, y_zoomout; // position of zoomout image
  // info image
  cv::Mat info;
  int h_info, w_info;       // size of info image
  int x_info, y_info;       // position of info image
  // show image (the big image that show in window)
  cv::Mat show; 
  int w_show, h_show;       // size of the show image
  int border;               // border width (gap)
  // mouse coord. 
  cv::Point2i mouseInShow;  // mouse pos. in show image coord. (must be integers)
  cv::Point2f mouseInUimg;  // mouse pos. in user image coord. (could be floats)
  cv::Point2f mouseInUimg_LastLbuttondown; // mouse pos. of last Left button-down 
  cv::Point2f pickedPoint;  // picked point in user image coord. (floats)
  bool mouseInZoomin;       // is the mouse in the zoomin window range
  bool mouseInZoomout;      // is the mouse in the zoomout window range
  bool mouseInInfo;         // is the mouse in the info window range
  bool zoominRangeMoving;   // is the zoominRange moving by the user
  // mouse event running state
  bool mouseEventRunning; 
  int  mouseEventHistory[10], mouseFlagsHistory[10];   
  // picked;
  bool picked;              
  // window name
  std::string winname; 
}; 

// internal functions (some are called by callback functions)

// This function prints user image size on the info image
int printUserImageInfo(struct PickPointSharedData & thisData)
{
  // display size info of user image
  // The display area is about Y:0~40 X:0~520 of info window
  char uimg_size[60];     // 
  cv::Point2i org(0, 0); // Upper-left pos. of text of image size info
  SPRINTF(uimg_size, "Image size (HxW): %04d x %04d", thisData.uimg.rows, thisData.uimg.cols); 
  int fontFace = FONT_HERSHEY_DUPLEX; 
      // options: FONT_HERSHEY_COMPLEX, FONT_HERSHEY_COMPLEX_SMALL, FONT_HERSHEY_DUPLEX,  
      //          FONT_HERSHEY_PLAIN, FONT_HERSHEY_SCRIPT_COMPLEX , FONT_HERSHEY_SCRIPT_SIMPLEX , 
      //          FONT_HERSHEY_SIMPLEX, FONT_HERSHEY_TRIPLEX 
  double fontScale = 1.; 
  cv::Scalar color = Scalar(255, 255, 0); // Blue, Green, Red
  int thickness = 1; 
  int lineType = 8; 
  bool bottomLeftOri = false; 
  int baseline = 0; 
  Size textSize = getTextSize(uimg_size, fontFace, fontScale, thickness, &baseline); 
  org.y += textSize.height + 3; 
  // clear image before putting text  
  Mat uimgInfoRoi(thisData.info(cv::Rect(0, 0, textSize.width, textSize.height + baseline + 3)));
  uimgInfoRoi.setTo(cv::Scalar(0, 0, 0));
  // put text
  cv::putText(thisData.info, uimg_size, org, fontFace, fontScale, 
	          color, thickness, lineType, bottomLeftOri);
  return 0; 
}

// This function prints mouse coord (in floats) wrt user image 
int updateMouseCoord(int event, int mx, int my, int flag, struct PickPointSharedData & thisData) {
  // display mouse coord (img) wrt user's image, can be floats
  // The display area is about Y:41~80 X:0~500 of info area
  
  // set mouseInShow (must be integers) in thisData
  thisData.mouseInShow.x = mx; 
  thisData.mouseInShow.y = my; 

  // clear mouseInZoomin, mouseInZoomout, mouseInInfo 
  thisData.mouseInZoomin  = false; 
  thisData.mouseInZoomout = false; 
  thisData.mouseInInfo    = false; 
  if (mx >= thisData.x_zoomin && mx < thisData.w_zoomin + thisData.x_zoomin && 
      my >= thisData.y_zoomin && my < thisData.h_zoomin + thisData.y_zoomin ) {
    thisData.mouseInZoomin = true;
  }
  if (mx >= thisData.x_zoomout && mx < thisData.w_zoomout + thisData.x_zoomout && 
      my >= thisData.y_zoomout && my < thisData.h_zoomout + thisData.y_zoomout ) {
    thisData.mouseInZoomout = true;
  }
  if (mx >= thisData.x_info && mx < thisData.w_info + thisData.x_info && 
      my >= thisData.y_info && my < thisData.h_info + thisData.y_info ) {
    thisData.mouseInInfo    = true;
  }

  // Clear the display area
//  Mat infoRoi(thisData.info(cv::Rect(0, 41, 1019, 40)));
  Mat infoRoi(thisData.info(cv::Rect(0, 41, 1019, 40)));
  infoRoi.setTo(cv::Scalar(30, 30, 30));

  // Check if mouse is within zoomin window
  if (thisData.mouseInZoomin == true) {
      // update mouseInZoomin
      thisData.mouseInZoomin = true; 
      // calculate image coord (thisData.mouseInUimg.x, thisData.mouseInUimg.y) 
      float zoomfactx, zoomfacty; 
      zoomfactx = 1.0f * thisData.w_zoomin / thisData.zoominRange.width; 
      zoomfacty = 1.0f * thisData.h_zoomin / thisData.zoominRange.height; 
      thisData.mouseInUimg.x = thisData.zoominRange.x - 0.5f + 0.5f / zoomfactx 
                             + (mx - thisData.x_zoomin) / zoomfactx; 
      thisData.mouseInUimg.y = thisData.zoominRange.y - 0.5f + 0.5f / zoomfacty 
                             + (my - thisData.y_zoomin) / zoomfacty;        
      // put text
      char coordText[60];     // 
      cv::Point2i org(0, 41); // Upper-left pos. of text
      SPRINTF(coordText, "Mouse image coord. in user image: X:%9.3f Y:%9.3f", 
              thisData.mouseInUimg.x, thisData.mouseInUimg.y); 
      int fontFace = FONT_HERSHEY_DUPLEX; 
      double fontScale = 1.; 
      cv::Scalar color = Scalar(255, 100, 200); // Blue, Green, Red
      int thickness = 1; 
      int lineType = 8; 
      bool bottomLeftOri = false; 
      int baseline = 0; 
      Size textSize = getTextSize(coordText, fontFace, fontScale, thickness, &baseline); 
      org.y += textSize.height + 3; 
      cv::putText(thisData.info, coordText, org, fontFace, fontScale, 
	          color, thickness, lineType, bottomLeftOri);
  }
  return 0; 
}

// This function prints user instruction on the info image. 
int printInstruction(struct PickPointSharedData & thisData)
{
  // display size info of user image
  // The display area is about Y:81~120 X:0~520 of info window
  cv::Point2i org(0, 120); 
  int fontFace = FONT_HERSHEY_DUPLEX; 
  double fontScale = 1. ; 
  cv::Scalar color = Scalar(200, 200, 200); // Blue, Green, Red
  int thickness = 1; 
  int lineType = 8; 
  bool bottomLeftOri = false; 
  cv::putText(thisData.info, "Instruction:", 
          cv::Point2i(0  , 120), fontFace, fontScale, color, thickness, lineType, bottomLeftOri);
  cv::putText(thisData.info, "Zoom-in the view: Mouse-left-button-click", 
          cv::Point2i(120, 150), fontFace, fontScale, color, thickness, lineType, bottomLeftOri);
  cv::putText(thisData.info, "Zoom-out the view: Ctrl mouse-left-button-click", 
          cv::Point2i(120, 180), fontFace, fontScale, color, thickness, lineType, bottomLeftOri);
  cv::putText(thisData.info, "Pan the view: Press mouse-left-button and move", 
          cv::Point2i(120, 210), fontFace, fontScale, color, thickness, lineType, bottomLeftOri);
  cv::putText(thisData.info, "Pick point: Mouse right-button-click. ", 
          cv::Point2i(120, 240), fontFace, fontScale, color, thickness, lineType, bottomLeftOri);
  return 0; 
}


int zoomAndUpdateZoominRange(struct PickPointSharedData & thisData, float furtherZoomFact) {
  int mx = thisData.mouseInShow.x; 
  int my = thisData.mouseInShow.y; 
  float fx = thisData.mouseInUimg.x;
  float fy = thisData.mouseInUimg.y;
  int   x0 = thisData.zoominRange.x;
  int   y0 = thisData.zoominRange.y;
  int   w0 = thisData.zoominRange.width;
  int   h0 = thisData.zoominRange.height; 
  float r; 
  int   x_new, y_new, w_new, h_new; 

  // check 
  if (furtherZoomFact > 0.f) {
    // calculate new range
    r = 1.f / furtherZoomFact;
    x_new = (int) (x0 * r + fx * (1.f - r) + 0.5f); 
    y_new = (int) (y0 * r + fy * (1.f - r) + 0.5f); 
    w_new = (int) (w0 * r + 0.5f); 
    h_new = (int) (h0 * r + 0.5f); 
  } else {
    x_new = 0;
    y_new = 0;
    w_new = thisData.uimg.cols;
    h_new = thisData.uimg.rows;
  }
  // new range
  cv::Rect newRange(x_new, y_new, w_new, h_new); 

  // zoom with transition
  smoothZoomAndShow(thisData.uimg, 
                             thisData.zoominRange, 
                             newRange, 
                             thisData.zoomin, 
                             thisData.show, 
                             thisData.winname, 
                             20, 1, cv::INTER_NEAREST, cv::INTER_AREA, 0); 
  // update zoominRange
  thisData.zoominRange = newRange; 
  // draw zoominRange in zoomout image
  drawZoominRangeInZoomout(thisData); 
  // imshow("zoomout", thisData.zoomout);
  // return;
  return 0;
}

int moveZoominRange(struct PickPointSharedData & thisData) {
  int   x_new, y_new, w_new, h_new; 
  x_new = thisData.zoominRange.x - (int) (thisData.mouseInUimg.x 
                                        - thisData.mouseInUimg_LastLbuttondown.x + 0.5f); 
  y_new = thisData.zoominRange.y - (int) (thisData.mouseInUimg.y
                                        - thisData.mouseInUimg_LastLbuttondown.y + 0.5f); 
  //x_new = thisData.zoominRange.x; 
  //y_new = thisData.zoominRange.y; 
  w_new = thisData.zoominRange.width; 
  h_new = thisData.zoominRange.height; 

  // new range
  cv::Rect newRange(x_new, y_new, w_new, h_new); 

  // zoom with transition
  smoothZoomAndShow(thisData.uimg, 
                             thisData.zoominRange, 
                             newRange, 
                             thisData.zoomin, 
                             thisData.show, 
                             thisData.winname, 
                             1, 1, cv::INTER_NEAREST, cv::INTER_AREA, 0); 
  // update zoominRange
  thisData.zoominRange = newRange; 
  // draw zoominRange in zoomout image
  drawZoominRangeInZoomout(thisData); 
  // imshow("zoomout", thisData.zoomout);
  // return;
  return 0;
}


int  drawZoominRangeInZoomout(struct PickPointSharedData & thisData) {
  float zoutFactX = thisData.zoomout.cols * 1.f / thisData.uimg.cols; 
  float zoutFactY = thisData.zoomout.rows * 1.f / thisData.uimg.rows; 
  cv::Rect zoominRangeWrtZoomoutCoord;
  zoominRangeWrtZoomoutCoord.x      = (int) (thisData.zoominRange.x      * zoutFactX + 0.5f); 
  zoominRangeWrtZoomoutCoord.y      = (int) (thisData.zoominRange.y      * zoutFactY + 0.5f); 
  zoominRangeWrtZoomoutCoord.width  = (int) (thisData.zoominRange.width  * zoutFactX + 0.5f); 
  zoominRangeWrtZoomoutCoord.height = (int) (thisData.zoominRange.height * zoutFactY + 0.5f); 
  cv::resize(thisData.uimg, 
	         thisData.zoomout, 
	         cv::Size(thisData.w_zoomout, thisData.h_zoomout), 0, 0, INTER_AREA); 
  rectangle(thisData.zoomout, zoominRangeWrtZoomoutCoord, cv::Scalar(255,100,100), 2, 8, 0); 
  return 0;
}

int  PickPointSharedData_Init(struct PickPointSharedData & thisData) {
  // init thisData
  thisData.mouseEventRunning = false; 
  thisData.picked = false; 
  for (int i = 0; i < 10; i++) {
    thisData.mouseEventHistory[i] = 0;
    thisData.mouseFlagsHistory[i] = 0;
  }
  thisData.mouseInZoomin  = false; 
  thisData.mouseInZoomout = false; 
  thisData.mouseInInfo    = false; 
  thisData.zoominRangeMoving = false; 
  return 0;
}



// callback functions 
void mouseEvent(int event, int x, int y, int flags, void *param)
{
  struct PickPointSharedData & thisData = *((struct PickPointSharedData*) param); 

  // check if mouse event has been running. If it is running, returns (to prevent double running).
  if (thisData.mouseEventRunning == true) return; 
  thisData.mouseEventRunning = true; 

  // update mouse event/flags history
  for (int i = 9; i > 0; i--) {
    thisData.mouseEventHistory[i] = thisData.mouseEventHistory[i - 1]; 
    thisData.mouseFlagsHistory[i] = thisData.mouseFlagsHistory[i - 1]; 
  }
  thisData.mouseEventHistory[0] = event; 
  thisData.mouseFlagsHistory[0] = flags; 

  // update mouse coord (mouseInShow, mouseInUimg, mouseInZoomin, mouseInZoomout, mouseInInfo)
  updateMouseCoord(event, x, y, flags, thisData); 

//  DEBUG print
  // printf("Check %d %d %d %d %d / %d %d %d %d %d\n", thisData.mouseInZoomin,
  //         thisData.mouseEventHistory[0], thisData.mouseEventHistory[1],
  //         thisData.mouseFlagsHistory[0], thisData.mouseFlagsHistory[1],
  //          true,
  //          cv::EVENT_LBUTTONUP, cv::EVENT_LBUTTONDOWN,
  //          0, cv::EVENT_FLAG_LBUTTON);
  // if ((flags & cv::EVENT_FLAG_ALTKEY  ) != 0) printf("flags == EVENT_FLAG_ALTKEY  \n");
  // if ((flags & cv::EVENT_FLAG_CTRLKEY ) != 0) printf("flags == EVENT_FLAG_CTRLKEY \n");
  // if ((flags & cv::EVENT_FLAG_LBUTTON ) != 0) printf("flags == EVENT_FLAG_LBUTTON \n");
  // if ((flags & cv::EVENT_FLAG_MBUTTON ) != 0) printf("flags == EVENT_FLAG_MBUTTON \n");
  // if ((flags & cv::EVENT_FLAG_RBUTTON ) != 0) printf("flags == EVENT_FLAG_RBUTTON \n");
  // if ((flags & cv::EVENT_FLAG_SHIFTKEY) != 0) printf("flags == EVENT_FLAG_SHIFTKEY\n");
  // if (event == cv::EVENT_LBUTTONDBLCLK) printf("event == EVENT_LBUTTONDBLCLK\n");
  // else if (event == cv::EVENT_LBUTTONDOWN  ) printf("event == EVENT_LBUTTONDOWN  \n");
  // else if (event == cv::EVENT_LBUTTONUP    ) printf("event == EVENT_LBUTTONUP    \n");
  // else if (event == cv::EVENT_MBUTTONDBLCLK) printf("event == EVENT_MBUTTONDBLCLK\n");
  // else if (event == cv::EVENT_MBUTTONDOWN  ) printf("event == EVENT_MBUTTONDOWN  \n");
  // else if (event == cv::EVENT_MBUTTONUP    ) printf("event == EVENT_MBUTTONUP    \n");
  // else if (event == cv::EVENT_RBUTTONDBLCLK) printf("event == EVENT_RBUTTONDBLCLK\n");
  // else if (event == cv::EVENT_RBUTTONDOWN  ) printf("event == EVENT_RBUTTONDOWN  \n");
  // else if (event == cv::EVENT_RBUTTONUP    ) printf("event == EVENT_RBUTTONUP    \n");
  // else if (event == cv::EVENT_MOUSEMOVE    ) printf("event == EVENT_MOUSEMOVE    \n");
  // else if (event == cv::EVENT_MOUSEWHEEL   ) printf("event == EVENT_MOUSEWHEEL   \n");
  // else                                       printf("event == %d\n", event);
  // printf("----------\n");

  // zoom-in (mouse click without CTRL)
  // if (thisData.mouseInZoomin == true &&
  //    thisData.mouseEventHistory[0] == cv::EVENT_LBUTTONUP &&
  //    thisData.mouseEventHistory[1] == cv::EVENT_LBUTTONDOWN ) {
  // if (thisData.mouseInZoomin == true &&
  //     thisData.mouseEventHistory[1] == cv::EVENT_LBUTTONDOWN &&
  //     thisData.mouseFlagsHistory[0] == 0 &&
  //     thisData.mouseFlagsHistory[1] == cv::EVENT_FLAG_LBUTTON ) {
  // if (thisData.mouseInZoomin == true &&
  //     thisData.mouseFlagsHistory[1] == 0 &&
  //     thisData.mouseEventHistory[1] == cv::EVENT_LBUTTONDOWN &&
  //     thisData.mouseFlagsHistory[0] == cv::EVENT_FLAG_LBUTTON &&
  //     thisData.mouseEventHistory[0] == cv::EVENT_LBUTTONUP ) {
  // if (thisData.mouseInZoomin == true &&
  //     (thisData.mouseFlagsHistory[1] &  cv::EVENT_FLAG_LBUTTON) == 0 &&
  //      thisData.mouseEventHistory[1] == cv::EVENT_LBUTTONDOWN        &&
  //     (thisData.mouseFlagsHistory[0] &  cv::EVENT_FLAG_LBUTTON) != 0 &&
  //     (thisData.mouseFlagsHistory[0] &  cv::EVENT_FLAG_CTRLKEY) == 0 &&
  //      thisData.mouseEventHistory[0] == cv::EVENT_LBUTTONUP ) {
  if (thisData.mouseInZoomin == true &&
      (thisData.mouseFlagsHistory[1] &  cv::EVENT_FLAG_LBUTTON) != 0 &&
       thisData.mouseEventHistory[1] == cv::EVENT_LBUTTONDOWN        &&
//      (thisData.mouseFlagsHistory[0] &  cv::EVENT_FLAG_LBUTTON) != 0 &&
      (thisData.mouseFlagsHistory[0] &  cv::EVENT_FLAG_CTRLKEY) == 0 &&
       thisData.mouseEventHistory[0] == cv::EVENT_LBUTTONUP ) {
    // zoom in
    zoomAndUpdateZoominRange(thisData, 2.0f); 
  }

  // zoom-out (mouse click with CTRL)
//  if (thisData.mouseInZoomin == true &&
//      thisData.mouseFlagsHistory[1] == cv::EVENT_FLAG_LBUTTON + cv::EVENT_FLAG_CTRLKEY &&
//      thisData.mouseEventHistory[1] == cv::EVENT_LBUTTONDOWN &&
//      thisData.mouseFlagsHistory[0] == cv::EVENT_FLAG_CTRLKEY &&
//      thisData.mouseEventHistory[0] == cv::EVENT_LBUTTONUP ) {
  if (thisData.mouseInZoomin == true &&
      (thisData.mouseFlagsHistory[1] &  cv::EVENT_FLAG_LBUTTON)  != 0 &&
       thisData.mouseEventHistory[1] == cv::EVENT_LBUTTONDOWN         &&
//      (thisData.mouseFlagsHistory[0] &  cv::EVENT_FLAG_LBUTTON ) != 0 &&
      (thisData.mouseFlagsHistory[0] &  cv::EVENT_FLAG_CTRLKEY ) != 0 &&
       thisData.mouseEventHistory[0] == cv::EVENT_LBUTTONUP ) {
    // zoom out
    zoomAndUpdateZoominRange(thisData, 0.5f); 
  }

  // mouse left button-down and move (start of moving) 
//  if (thisData.mouseInZoomin == true &&
//      thisData.mouseFlagsHistory[1] == cv::EVENT_FLAG_LBUTTON &&
//      thisData.mouseEventHistory[1] == cv::EVENT_LBUTTONDOWN &&
//      thisData.mouseFlagsHistory[0] == cv::EVENT_FLAG_LBUTTON &&
//      thisData.mouseEventHistory[0] == cv::EVENT_MOUSEMOVE ) {
  if (thisData.mouseInZoomin == true &&
//      (thisData.mouseFlagsHistory[1] &  cv::EVENT_FLAG_LBUTTON) == 0 &&
       thisData.mouseEventHistory[1] == cv::EVENT_LBUTTONDOWN        &&
      (thisData.mouseFlagsHistory[0] &  cv::EVENT_FLAG_LBUTTON) != 0 &&
       thisData.mouseEventHistory[0] == cv::EVENT_MOUSEMOVE ) {
    // save the current mouse coord. (in uimg) and start the moving
    thisData.mouseInUimg_LastLbuttondown.x = thisData.mouseInUimg.x;
    thisData.mouseInUimg_LastLbuttondown.y = thisData.mouseInUimg.y;
    thisData.zoominRangeMoving = true;  // start moving. (will stop when Lbutton up) 
  }


  // moving (mouse button-down and move before button-up) 
  if (thisData.mouseInZoomin == true &&
      thisData.zoominRangeMoving == true && 
      (thisData.mouseFlagsHistory[0] & cv::EVENT_FLAG_LBUTTON) != 0 &&
       thisData.mouseEventHistory[0] == cv::EVENT_MOUSEMOVE ) {
    // moving
    moveZoominRange(thisData); 
  }

  // mouse L button up (stop moving)
  if ( (thisData.mouseFlagsHistory[0] & cv::EVENT_FLAG_LBUTTON) == 0 ) {
    // stop moving the zoominRange 
    thisData.zoominRangeMoving = false; 
  }
  
  // picked
  if (thisData.mouseInZoomin == true &&
      event == cv::EVENT_RBUTTONUP && 
      !(flags & EVENT_FLAG_CTRLKEY) ) {
    thisData.picked = true;
    thisData.pickedPoint.x = thisData.mouseInUimg.x; 
    thisData.pickedPoint.y = thisData.mouseInUimg.y; 
    zoomAndUpdateZoominRange(thisData, 0.0f); 
  }

  imshow(thisData.winname, thisData.show); 

  // return 
  thisData.mouseEventRunning = false; 
}


int pickAPoint(const std::string & winName, 
                        const cv::Mat & userImg, 
                        cv::Point2f & pickedPoint, 
                        int breakKey, int confirmKey1, int confirmKey2) {
  
  // check parameters
  if (userImg.cols < 1 || userImg.rows < 1) {
//    fprintf(stderr, "Warning: pickAPoint() got an empty base image. Ignored picking point action.\n");
    return -1; 
  }

  // Most of the window data, image data, size/position data are in thisData
  struct PickPointSharedData thisData; 

  // init thisData
  PickPointSharedData_Init(thisData); 

  // set window name
  thisData.winname = winName; 

  // get user image 
  thisData.uimg = userImg;
  thisData.h_uimg = thisData.uimg.rows; 
  thisData.w_uimg = thisData.uimg.cols; 

  // layout of zoomin window
  thisData.border = 10;          // border width
  thisData.w_zoomin = MAX_IMAGE_WIN_WIDTH; // Width of zoomin window
  thisData.h_zoomin = thisData.h_uimg * thisData.w_zoomin / thisData.w_uimg; 
  if (thisData.h_zoomin > MAX_IMAGE_WIN_HEIGHT) {
    thisData.h_zoomin = MAX_IMAGE_WIN_HEIGHT; 
    thisData.w_zoomin = thisData.h_zoomin * thisData.w_uimg / thisData.h_uimg; 
  }
  thisData.x_zoomin = 0 + thisData.border; 
  thisData.y_zoomin = 0 + thisData.border; 
  // zoomin range
  thisData.zoominRange = cv::Rect(0, 0, thisData.w_uimg, thisData.h_uimg); 

  // zoomout window of user image
  thisData.w_zoomout = thisData.w_zoomin;      // Width of zoomout window
  thisData.h_zoomout = thisData.h_uimg * thisData.w_zoomout / thisData.w_uimg; 
  thisData.x_zoomout = thisData.x_zoomin + thisData.w_zoomin + thisData.border * 2; 
  thisData.y_zoomout = 0 + thisData.border; 

  // info window 
  thisData.h_info = 400;         // Height of infomation region of show window (fixed)
  thisData.w_info = thisData.w_zoomin + thisData.border * 2 + thisData.w_zoomout;
  if (thisData.w_info < 1020) thisData.w_info = 1020; 
  thisData.x_info = thisData.border; 
  thisData.y_info = thisData.border + std::max(thisData.h_zoomin, thisData.h_zoomout)
	              + thisData.border; 

  // size of show (overall) image
  thisData.w_show = thisData.border + thisData.w_zoomin + thisData.border *2 
	              + thisData.w_zoomout + thisData.border; 
  if (thisData.w_show < 1040) thisData.w_show = 1040; 
  thisData.h_show = thisData.border + std::max(thisData.h_zoomin, thisData.h_zoomout) 
	              + thisData.border + thisData.h_info; 

  // create Mats and their links
  thisData.show = cv::Mat(thisData.h_show, thisData.w_show, thisData.uimg.type(), cv::Scalar(0,0,0)); 
  thisData.zoomin  = thisData.show(cv::Rect(thisData.x_zoomin,  thisData.y_zoomin,
	                                        thisData.w_zoomin,  thisData.h_zoomin ));
  thisData.zoomout = thisData.show(cv::Rect(thisData.x_zoomout, thisData.y_zoomout, 
	                                        thisData.w_zoomout, thisData.h_zoomout)); 
  thisData.info    = thisData.show(cv::Rect(thisData.x_info,    thisData.y_info,    
	                                        thisData.w_info,    thisData.h_info)); 

  // Resize user image 
  cv::resize(thisData.uimg, 
	         thisData.zoomout, 
	         cv::Size(thisData.w_zoomout, thisData.h_zoomout), 0, 0, INTER_AREA); 
  cv::resize(thisData.uimg(thisData.zoominRange), 
	         thisData.zoomin, 
	         cv::Size(thisData.w_zoomin,  thisData.h_zoomin),  0, 0, INTER_AREA);

  // put info into show image
  printUserImageInfo(thisData);  
  printInstruction(thisData); 

  // draw zoomin range in the zoomout window
  drawZoominRangeInZoomout(thisData); 

  // create window and show
  cv::namedWindow(winName, CV_WINDOW_AUTOSIZE); 
  cv::imshow(winName, thisData.show);

  // set callback function
  cv::setMouseCallback(winName, mouseEvent, (void*) &thisData); 

  // loop 
  int key; 
  do {
    key = waitKey(10); 

    if (thisData.picked == true) {
      pickedPoint.x = thisData.pickedPoint.x;
      pickedPoint.y = thisData.pickedPoint.y;
	  if (confirmKey1 == 0 && confirmKey2 == 0) break;
	  if (key > 0 && (key == confirmKey1 || key == confirmKey2)) break;
    }

  } while (key != breakKey); 

  cv::destroyWindow(winName);

  // if not picked (program stopped by user pressing breakKey)
  if (thisData.picked == false) return breakKey; 

  return 0;
}


int pickATemplate(const std::string & winName,
	const cv::Mat & userImg,
	cv::Point2f & pickedPoint,
	cv::Rect & pickedRoi,
	int breakKey, int confirmKey1, int confirmKey2,
	cv::Size initRoiSize)
{
	cv::Rect tmpltRect; 
	cv::Point2f refPoint; 
	// pick the point
	int pickPointReturnValue = pickAPoint(winName, userImg, pickedPoint, breakKey, confirmKey1, confirmKey2);
	// define a large range for user to select template range (rect)
	tmpltRect = getTmpltRectFromImageSize(userImg.size(), pickedPoint, initRoiSize, refPoint);
	cv::Mat userImgTmpltRect; 
	// draw a cross on the point in the large range 
	cv::Point2f pointOnLargeRect(pickedPoint.x - tmpltRect.x, pickedPoint.y - tmpltRect.y);
	userImg(tmpltRect).copyTo(userImgTmpltRect);
	drawPointOnImage(userImgTmpltRect, pointOnLargeRect, string("x"), 8, 1, cv::Scalar(0,255,0), .5f, -1, 0); 
	// resize userImgTmpltRect 
	double fact = 2.5;
	cv::resize(userImgTmpltRect, userImgTmpltRect, cv::Size(0, 0), fact, fact, cv::InterpolationFlags::INTER_AREA);
	// Ask user to select the rect of the template
	pickedRoi = cv::selectROI("Select range (rect) of template", userImgTmpltRect, false, false);
	pickedRoi.x = (int) (pickedRoi.x / fact + tmpltRect.x + .5);
	pickedRoi.y = (int) (pickedRoi.y / fact + tmpltRect.y + .5);
	pickedRoi.width = (int) (pickedRoi.width / fact + .5); 
	pickedRoi.height = (int)(pickedRoi.height / fact + .5);
	cv::destroyWindow("Select range (rect) of template"); 
	return 0;
}

