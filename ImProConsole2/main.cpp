#include <iostream>
#include <cmath>
#include <opencv2/opencv.hpp>

#include "improStrings.h"

#include "Submenu.h"
#include "CamMoveCorrector.h"

using std::string;
using std::cout;
using std::endl;
using std::vector;

int FuncCalibStereoOnSite(int, char**);
int FuncCalibInLabOnSite(int, char**);
int FuncCalibOnSiteUserPoints(int, char**);
int FuncCalibOnlyExtrinsic(int, char**);

int FuncVideo2Pics(int argc, char** argv);
int FuncPics2Video(int argc, char** argv);
int FuncVideo2PicsResize(int argc, char** argv);
int FuncPics2VideoResize(int argc, char** argv);
int FuncPip(int argc, char** argv);

int FuncTemplatesPicking(int argc, char** argv);
int FuncQ4TemplatesPicking(int argc, char** argv);

int FuncCamMoveCorrection(int argc, char** argv);

int FuncTrackingPointsEcc(int argc, char** argv);
int FuncTrackingPyrTmpltMatch(int argc, char** argv);

int FuncSyncTwoCams(int argc, char** argv);

int FuncTriangulationAllSteps(int argc, char** argv);

int FuncDrawGrid(int argc, char ** argv);
int FuncDrawHouse(int argc, char ** argv);
int FuncDrawRotatingTemplate(int argc, char ** argv);

int FuncOptflowSeq(int argc, char ** argv);

int FuncWallDisp(int argc, char ** argv);    // made for SPD test 2019
int FuncWallDispCam(int argc, char ** argv); // made for SPD test 2019
int FuncStoryDisp(int argc, char ** argv); // made for inter-story calculation
int FuncStoryDispV2(int argc, char** argv); // made for inter-story calculation
int FuncStoryDispV3(int argc, char** argv); // V2 + writing each frame (optionally)
int FuncStoryDispV4(int argc, char** argv); // Eric V2 + wider big table
int FuncStoryDispV6(int argc, char** argv); // Eric V6 (reading image file with named with date/time)
int FuncStoryDispV7(int argc, char** argv); // V7 (modified V6 in Vincent style, including printing "#", can specify image directory, and some details)
int FuncStoryDispV8(int argc, char** argv); // V8 (final version in Jun 2020, disabled template match and ECC tracking methods, left only optical flow)

int FuncBucklingRestrainedBrace_url(int argc, char ** argv); // made for BRB test 2019

int FuncTryCamFocusExposure(int argc, char ** argv);

int FuncVideoDenseTracking(int argc, char ** argv);
int FuncVidImPointsQ4(int argc, char** argv);
int FuncVidOptflowToVelocity(int argc, char** argv);
int FuncMandelbrot(int argc, char** argv);

int FuncSingleCamWallMonitoring(int argc, char** argv);
int FuncWallSingleCam(int argc, char** argv);
int FuncImshowOnline(int argc, char** argv);

int FuncConstraintOnPolySurface(int argc, char** argv); // made for DSIVC PFPI isolator tests

int FuncTestPlotCamRotNewDisp(int argc, char** argv); // made for DSIVC PFPI isolator tests

int FuncUndistortOnline(int argc, char** argv);

int FuncDestroyAllWindows(int argc, char** argv){cv::destroyAllWindows();return 0;}


using namespace cv;

int q4MembraneStrains_test();

int main(int argc, char ** argv)
{
//    std::string onlyFilename = "123456789012345678901234567890";
//    printf(" ... %s", onlyFilename.substr(onlyFilename.length() - 15, 15).c_str());
//    printf("\n"); return 0;
    ////////////////////////////
    Submenu s(argc, argv);
    s.addItem("calsite",    "Calibration: Stereo calibration on site               ", FuncCalibStereoOnSite);
    s.addItem("callabsite", "Calibration: Intrinsic in Lab. Extrinsic on site      ", FuncCalibInLabOnSite);
    s.addItem("calupoints", "Calibration: Intrinsic/Extrisic on site by user points", FuncCalibOnSiteUserPoints);
    s.addItem("calexonly",  "Calibration: Given intrinsic data and run extrinsic calibration", FuncCalibOnlyExtrinsic);

    s.addItem("v2p",        "Conversion: Convert video to pictures", FuncVideo2Pics);
    s.addItem("p2v",        "Conversion: Convert pictures to video", FuncPics2Video);
    s.addItem("v2pResize",  "Conversion: Convert video to pictures with resizing", FuncVideo2PicsResize);
    s.addItem("p2vResize",  "Conversion: Convert pictures to video with resizing", FuncPics2VideoResize);
    s.addItem("pip", "Conversion: Picture-In-Picture Integration", FuncPip);

    s.addItem("picktm",     "Picking: Templates Picking",     FuncTemplatesPicking);
    s.addItem("picktmq4",   "picking: Generating quadrilateral mesh of templates by picking four corners.", FuncQ4TemplatesPicking);

    s.addItem("cammov",     "Cam correction: Camera movement correction (pic 2 pic, ref. points tracked) ", FuncCamMoveCorrection);

    s.addItem("ecc",        "Tracking: Track Points Using ECC method",                FuncTrackingPointsEcc);
    s.addItem("tmatch",     "Tracking: Track by pyramid template match",              FuncTrackingPyrTmpltMatch);

    s.addItem("syncC2",     "Synchronize Camera 2 to match Camera 1",                 FuncSyncTwoCams);

    s.addItem("trianga",    "Triangulation (all steps a file)",                       FuncTriangulationAllSteps);

    s.addItem("drawgrid",   "Draw a grid on a given image.", FuncDrawGrid);
    s.addItem("drawhouse",  "Draw a house allowing tuning intrinsic p.",               FuncDrawHouse);
    s.addItem("drawRotTmplt", "Draw rotating template and create a file sequence.", FuncDrawRotatingTemplate);

    s.addItem("optflow", "Dense (Farneback) optical flow analysis", FuncOptflowSeq);

    s.addItem("wallDisp", "Plane wall displacement analysis (online)", FuncWallDisp);
    s.addItem("wallDispCam", "Plane wall displacement analysis (online, webcam)", FuncWallDispCam);
    s.addItem("storyDisp", "Measure story drift (online, Raspberry)", FuncStoryDisp);
    s.addItem("storyDispV2", "Measure story drift (online, Raspberry V2)", FuncStoryDispV2);
    s.addItem("storyDispV4", "Measure story drift (V2 + Wider Table)", FuncStoryDispV4);
    s.addItem("storyDispV6", "Measure story drift (Eric V6, reading file named with date/time)", FuncStoryDispV6);
    s.addItem("storyDispV7", "Measure story drift (V6 enhanced)", FuncStoryDispV7);
    s.addItem("storyDispV8", "Measure story drift (disables template-match and ECC)", FuncStoryDispV8);

//    s.addItem("brburl", "FuncBucklingRestrainedBrace_url (for BRB test 2019)", FuncBucklingRestrainedBrace_url);

    s.addItem("tryCam", "Try the best camera settings of focus and exposure", FuncTryCamFocusExposure);

    s.addItem("vidTrack", "Video dense-point tracking", FuncVideoDenseTracking);
    s.addItem("vidImPointsQ4", "Interpolate q4 points through vidTrack result.", FuncVidImPointsQ4);
    s.addItem("vidOptflowToVelocity", "Convert optical flow (image velocity) to real velocity.", FuncVidOptflowToVelocity);
    s.addItem("mandelbrot", "Mandelbrot Set plotting.", FuncMandelbrot);

//	s.addItem("1camwall", "Monitor a wall (polygon) by using one camera.", FuncSingleCamWallMonitoring);
    s.addItem("1camwall", "Monitor a wall (polygon) by using one camera.", FuncWallSingleCam);
    s.addItem("imshowonline", "Monitor a wall (polygon) by using one camera.", FuncImshowOnline);

    s.addItem("undistonline", " Undistort images from a file sequence and outputs to another.", FuncUndistortOnline);
    s.addItem("destroywin", " Call OpenCV cv::destroyAllWindows().", FuncDestroyAllWindows);
    s.addItem("polySurf", " (Underconstruction) Tracking points on a constrained polynomial surface.", FuncConstraintOnPolySurface);



//    s.addItem("test0", "Just for test.", FuncTest0);

    s.addItem("testPlot1", "Plotting camRot and newDisp.", FuncTestPlotCamRotNewDisp);

    s.run();

    return 0;
}
