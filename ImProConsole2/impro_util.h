#pragma once

#include <vector>
#include <string>
#include <iostream>
#include <fstream>
#include "opencv2/core.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/video.hpp"
#include "opencv2/objdetect.hpp"
#include "opencv2/opencv.hpp"

#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884
#endif
//#define M_PIl 3.141592653589793238462643383279502884L

#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
#else
typedef int errno_t;
int fopen_s(FILE **f, const char *name, const char *mode);
#endif


double getCpusTime();
double getWallTime();

bool findChessboardCornersSubpix(cv::Mat image, cv::Size patternSize, std::vector<cv::Point2f> & corners,
	int flags = cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE,
	cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-6));

bool findChessboardCornersSubpix(cv::Mat image, cv::Size patternSize, cv::Mat & corners,
	int flags = cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE,
	cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-6));

std::vector<cv::Point3f> create3DChessboardCorners(cv::Size bsize, float squareSize_w, float squareSize_h);
cv::Mat create3DChessboardCornersMat(cv::Size bsize, float squareSize_w, float squareSize_h);

double calibrateCameraFromChessboardImages(
	const std::vector<cv::Mat> & imgs,
	cv::Size imageSize,
	cv::Size bsize,
	float squareSize_w, float squareSize_h,
	cv::Mat & cmat,
	cv::Mat & dmat,
	std::vector<cv::Mat> & rmats,
	std::vector<cv::Mat> & tmats,
	cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-6),
	int flags = 0  // flag of calibrateCamera() 
);


double stereoCalibrateChessboard(
	const std::vector<cv::Mat> & imgs_L,
	const std::vector<cv::Mat> & imgs_R,
	cv::Size imageSize,
	cv::Size bsize,
	float squareSize_w, float squareSize_h,
	cv::Mat & cmat_L,
	cv::Mat & cmat_R,
	cv::Mat & dmat_L,
	cv::Mat & dmat_R,
	cv::Mat & rmat,
	cv::Mat & tmat,
	cv::Mat & emat,
	cv::Mat & fmat,
	cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 1e-6),
	int flags = 0  // flag of stereoCalibrate() 
);

cv::Mat Rvec2R4(const cv::Mat & rvec, const cv::Mat & tvec);

// getTmpltFromImage() gives you a template by copying a sub-image from a full (large) image, 
// template size, and a given image point. 
// The template is a subimage of the full image, and the reference point of the template is the same 
// as the pnt point of the full image. 
// The reference point basically is near the center of the tmplt. Since the pnt could be any float 
// between two integers, the reference point contains floats. 
// If the template is partially out of the range of full image, reference point will get more farther 
// away from the center. 

int getTmpltFromImage(
	const cv::Mat       &    img,
	const cv::Point2f   &    pnt,
	const cv::Size      &    tmpltSize,
	cv::Mat       &    tmplt,
	cv::Point2f   &    ref);

cv::Rect getTmpltRectFromImage(
	const cv::Mat       &    img,
	const cv::Point2f   &    pnt,
	const cv::Size      &    tmpltSize,
	cv::Point2f   &    ref);

cv::Rect getTmpltRectFromImageSize(
	const cv::Size      &    imgSize,
	const cv::Point2f   &    pnt,
	const cv::Size      &    tmpltSize,
	cv::Point2f   &    ref); // 

cv::Rect getTmpltRectFromImageSizeWithPreferredRef(
	const cv::Size      &    imgSize,
	const cv::Point2f   &    pnt,
	const cv::Size      &    tmpltSize,
	cv::Point2f   &    ref); // Preferred reference point, which does not need to be at the center of template. If template is not near border of full image, ref will remain. 

//#if defined(WIN32) || defined(_WIN32) || defined(__WIN32) && !defined(__CYGWIN__)
std::string              uigetfile(void);
std::vector<std::string> uigetfiles(void);
std::string              uigetdir(void); 
std::string              uiputfile(void);
//#endif

std::string appendSubstringBeforeLastDot(std::string fname, std::string substr);

//! mtm_ecc positions a template by running multilevel template match and ecc
/*!
\details 
mtm_ecc positions a target by running multilevel template match 
(i.e., matchTemplateWithRotPyr) with rough precision and adjust the precision by
running ecc. 
\param imgSrch search image 
\param imgInit initial image (which contains the template)
\param tPoint position of the target in the initial image
\param tSize template size of the target
\param result result ([0]:x, [1]:y, [2]:rot, [3]:coeff(1.0 for best), [4]:total cpus time, [5]:total wall time, [6]:large-win cpus time, [7]:large-win wall time, [8]:small-win cpus time, [9]:small-win wall time, [10]:ecc cpus time, [11]:ecc wall time
\param tGuess initial guess of the target in the search image (default: tPoint)
\param xMin minimum possible value of x (default: tPoint.x - 2 * tSize.w)
\param xMax maximum possible value of x (default: tPoint.x + 2 * tSize.w)
\param yMin minimum possible value of y (default: tPoint.y - 2 * tSize.h)
\param yMax maximum possible value of y (default: tPoint.y + 2 * tSize.h)
\param rotMin miminum possible value of rotation (in degree) (default: 0.0f)
\param rotMax maxinum possible value of rotation (in degree) (default: 0.0f)
\param largeWinSize large window size (size which can cover repeating pattern around. default: tSize * 2)
*/
int mtm_ecc(cv::InputArray _image, cv::InputArray _tmplt,
	cv::Point2f tPoint, cv::Size tSize,
	std::vector<double> & result,
	cv::Point2f tGuess = cv::Point2f(std::nanf(""), std::nanf("")),
	float rotGuess = std::nanf(""), 
	float xMin = std::nanf(""), float xMax = std::nanf(""),
	float yMin = std::nanf(""), float yMax = std::nanf(""),
	float rotMin = 0.0f, float rotMax = 0.0f, 
	cv::Size largeWinSize = cv::Size(-1, -1));

//! mtm_opf positions a template by running multilevel template match and optical flow
/*!
\details
mtm_opf positions a target by running multilevel template match
(i.e., matchTemplateWithRotPyr) with rough precision and adjust the precision by
running optical flow.
\param imgSrch search image
\param imgInit initial image (which contains the template)
\param tPoint position of the target in the initial image
\param tSize template size of the target
\param result result ([0]:x, [1]:y, [2]:rot, [3]:coeff(1.0 for best), [4]:total cpus time, [5]:total wall time, [6]:large-win cpus time, [7]:large-win wall time, [8]:small-win cpus time, [9]:small-win wall time, [10]:opf cpus time, [11]:opf wall time
\param tGuess initial guess of the target in the search image (default: tPoint)
\param xMin minimum possible value of x (default: tPoint.x - 2 * tSize.w)
\param xMax maximum possible value of x (default: tPoint.x + 2 * tSize.w)
\param yMin minimum possible value of y (default: tPoint.y - 2 * tSize.h)
\param yMax maximum possible value of y (default: tPoint.y + 2 * tSize.h)
\param rotMin miminum possible value of rotation (in degree) (default: 0.0f)
\param rotMax maxinum possible value of rotation (in degree) (default: 0.0f)
\param largeWinSize large window size (size which can cover repeating pattern around. default: tSize * 2)
*/
//int mtm_opf(cv::InputArray _image, cv::InputArray _tmplt,
//	cv::Point2f tPoint, cv::Size tSize,
//	std::vector<double> & result,
//	cv::Point2f tGuess = cv::Point2f(std::nanf(""), std::nanf("")),
//	float rotGuess = std::nanf(""),
//	float xMin = std::nanf(""), float xMax = std::nanf(""),
//	float yMin = std::nanf(""), float yMax = std::nanf(""),
//	float rotMin = 0.0f, float rotMax = 0.0f,
//	cv::Size largeWinSize = cv::Size(-1, -1));

//! mtm_opfs positions multiple templates by running multilevel template match with rotations
//  and optical flow
/*!
\details
mtm_opfs positions multiple targets by running multilevel template match
(i.e., matchTemplateWithRotPyr) with rough precision and adjust the precision by
running optical flow. The function interface is similar to calcOpticalFlowPyrLK().
\param imgInit initial image (which contains the templates)
\param imgSrch search image
\param tPointsInit vector of targets (Point2f) in the initial image
\param tPointsSrch vector of targets (Point3f, i.e., x, y and rotation in degree) in the search image (input: guess, output: analyzed) 
\param maxMove vector (of float) of maximum movement along x, y, and rotation (in degree). maxMove[2] == 0.0f indicates rotation is not tracked.
\param status output status vector of uchar. 1 for found. 0 for otherwise.
\param error output error vector of float. 
\param timing output timing data. [0]:total cpus time, [1]:total wall time, [2]:large-win cpus time, [3]:large-win wall time, [4]:small-win cpus time, [5]:small-win wall time, [6]:opf cpus time, [7]:opf wall time
\param winSize window size (or template size) of targets 
\param maxLevel For mtm: size factor of winSize in rough matching (step 1). For optical flow, maximum pyramid level number. 0:same size, 1:double (x2), 2:(x4), 3:(x8)
\param criteria specifying the termination criteria of the iterative search algorithm
\param flags OPTFLOW_USE_INITIAL_FLOW, OPTFLOW_LK_GET_MIN_EIGENVALS. 
\return 0:success. -1:empry image(s). -2:no valid initial point. 
*/
int mtm_opfs(cv::Mat imgInit, cv::Mat imgSrch,
	const std::vector<cv::Point2f> & tPointsInit,
	std::vector<cv::Point3f> & tPointsSrch,
	const std::vector<float> & maxMove,
	std::vector<uchar> & status,
	std::vector<float> & error,
	std::vector<float> & timing, 
	cv::Size winSize = cv::Size(25, 25), 
	int maxLevel = 3,
	cv::TermCriteria criteria = cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01),
	int flags = cv::OPTFLOW_USE_INITIAL_FLOW);

int points2fVecValid(const std::vector<cv::Point2f> & oldVec,
	std::vector<cv::Point2f> & newVec,
	std::vector<int> & o2n, std::vector<int> & n2o,
	float xMin = -1e38, float xMax = 1e38,
	float yMin = -1e38, float yMax = 1e38); 
int points3fVecValid(const std::vector<cv::Point3f> & oldVec,
	std::vector<cv::Point3f> & newVec,
	std::vector<int> & o2n, std::vector<int> & n2o,
	float xMin = -1e38, float xMax = 1e38,
	float yMin = -1e38, float yMax = 1e38,
	float zMin = -1e38, float zMax = 1e38);
bool isValid(cv::Point2f p);
bool isValid(cv::Point3f p);
bool isValid(const std::vector<cv::Point2f> & vp);

std::string appendSlashOrBackslashAfterDirectoryIfNecessary(std::string dir); 
std::string extFilenameRemoved(std::string f);
std::string directoryOfFullPathFile(std::string f);
std::string fileOfFullPathFile(std::string f);


template <class T>
int VecValid(const std::vector<T> & oldVec,
	std::vector<T> & newVec,
	std::vector<int>& o2n, std::vector<int>& n2o)
{
	o2n.resize(oldVec.size(), -1);
	int nValid = 0;
	for (int i = 0; i < oldVec.size(); i++) {
		if (isValid(oldVec[i])) {
			o2n[i] = nValid;
			nValid++;
		}
	}
	n2o.resize(nValid);
	for (int i = 0; i < oldVec.size(); i++)
		if (o2n[i] >= 0)
			n2o[o2n[i]] = i; 

	if (nValid == 0) return -1;
	newVec.resize(nValid);
	for (int i = 0; i < nValid; i++)
		newVec[i] = oldVec[n2o[i]];
	return 0;
}

int paintMarkersOnPointsInImage(
	const cv::Mat & inImg, cv::Mat & outImg,
	const std::vector<cv::Point2f> & points,
	int draw_type, int mark_size); 


//! interpQ4() generates refined points within a quadrangle defined by four given points.
/*!
\details This function generates refined points with a quadrangle defined by four given points.
The interpolation method is based on perspective view with homography matrix. 
\param inPoints four points of the quadragle
\param n12 number of refined points along point-1 and point-2. 
\param n23 number of refined points along point-2 and point-3.
\return the refined (interpolated) points.
*/
std::vector<cv::Point2f> interpQ4(const std::vector<cv::Point2f> & inPoints, int n12, int n23); 

//! interpQ43d() generates refined points within a quadrangle defined by four given 3D points.
/*!
\details This function generates refined points with a quadrangle defined by four given 3D points.
The interpolation method is based on bi-linear interpolation.
\param inPoints four points of the quadragle
\param n12 number of refined points along point-1 and point-2.
\param n23 number of refined points along point-2 and point-3.
\return the refined (interpolated) points.
*/
std::vector<cv::Point3d> interpQ43d(const std::vector<cv::Point3d> & inPoints, int n12, int n23);

//! Read data from cin but skip complete line if a string starts from '#'
int readIntFromCin(int lowerBound = -2147483646, int higherBound = 2147483647);
double readDoubleFromCin(double lowerBound = -1.e307, double higherBound = 1.e307);
std::string readStringFromCin();
std::string readStringLineFromCin();
int readIntFromIstream(std::istream &, int lowerBound = -2147483646, int higherBound = 2147483647);
double readDoubleFromIstream(std::istream &, double lowerBound = -1.e307, double higherBound = 1.e307);
std::string readStringFromIstream(std::istream &);
std::string readStringLineFromIstream(std::istream &);
std::vector<std::string> readStringsFromStringLine(std::string); 

// try all fourcc for video writer
int preferredFourcc(); 

//! draw points on image
/*!
\param img the image to be drawn
\param points points positions in format of Mat(N,2,Point2f)
\param symbol symbol of markers, can be "+", "x", "o", "square"
\param size the size of the markers (in pixels)
\param thickness the thickness of markers (in pixels)
\param color cv::Scalar(blue,green,red)
\param alpha alpha of markers (0:invisible, 1:opaque)
\param putText put index near the point (<0 for not putting any index, >= 0 to be the first index)
\param shift shift for subpixel drawing
*/
int drawPointsOnImage(cv::Mat & img, cv::Mat points, std::string symbol = std::string("o"),
	int size = 8, int thickness = 1, 
	cv::Scalar color = cv::Scalar(0, 255, 0),
	float alpha = 0.5f, int putText = -1, int shift = 3); 

int drawPointOnImage(cv::Mat & img, cv::Point2f point, std::string symbol = std::string("o"),
	int size = 8, int thickness = 1,
	cv::Scalar color = cv::Scalar(0, 255, 0),
	float alpha = 0.5f, int putText = -1, int shift = 3);

const unsigned char jet_bgr[256][3] = { {131,0,0},{135,0,0},{139,0,0},{143,0,0},{147,0,0},{151,0,0},{155,0,0},{159,0,0},
{163,0,0},{167,0,0},{171,0,0},{175,0,0},{179,0,0},{183,0,0},{187,0,0},{191,0,0},{195,0,0},{199,0,0},{203,0,0},{207,0,0},
{211,0,0},{215,0,0},{219,0,0},{223,0,0},{227,0,0},{231,0,0},{235,0,0},{239,0,0},{243,0,0},{247,0,0},{251,0,0},{255,0,0},
{255,4,0},{255,8,0},{255,12,0},{255,16,0},{255,20,0},{255,24,0},{255,28,0},{255,32,0},{255,36,0},{255,40,0},{255,44,0},
{255,48,0},{255,52,0},{255,56,0},{255,60,0},{255,64,0},{255,68,0},{255,72,0},{255,76,0},{255,80,0},{255,84,0},{255,88,0},
{255,92,0},{255,96,0},{255,100,0},{255,104,0},{255,108,0},{255,112,0},{255,116,0},{255,120,0},{255,124,0},{255,128,0},
{255,131,0},{255,135,0},{255,139,0},{255,143,0},{255,147,0},{255,151,0},{255,155,0},{255,159,0},{255,163,0},{255,167,0},
{255,171,0},{255,175,0},{255,179,0},{255,183,0},{255,187,0},{255,191,0},{255,195,0},{255,199,0},{255,203,0},{255,207,0},
{255,211,0},{255,215,0},{255,219,0},{255,223,0},{255,227,0},{255,231,0},{255,235,0},{255,239,0},{255,243,0},{255,247,0},
{255,251,0},{255,255,0},{251,255,4},{247,255,8},{243,255,12},{239,255,16},{235,255,20},{231,255,24},{227,255,28},
{223,255,32},{219,255,36},{215,255,40},{211,255,44},{207,255,48},{203,255,52},{199,255,56},{195,255,60},{191,255,64},
{187,255,68},{183,255,72},{179,255,76},{175,255,80},{171,255,84},{167,255,88},{163,255,92},{159,255,96},{155,255,100},
{151,255,104},{147,255,108},{143,255,112},{139,255,116},{135,255,120},{131,255,124},{128,255,128},{124,255,131},
{120,255,135},{116,255,139},{112,255,143},{108,255,147},{104,255,151},{100,255,155},{96,255,159},{92,255,163},
{88,255,167},{84,255,171},{80,255,175},{76,255,179},{72,255,183},{68,255,187},{64,255,191},{60,255,195},{56,255,199},
{52,255,203},{48,255,207},{44,255,211},{40,255,215},{36,255,219},{32,255,223},{28,255,227},{24,255,231},{20,255,235},
{16,255,239},{12,255,243},{8,255,247},{4,255,251},{0,255,255},{0,251,255},{0,247,255},{0,243,255},{0,239,255},
{0,235,255},{0,231,255},{0,227,255},{0,223,255},{0,219,255},{0,215,255},{0,211,255},{0,207,255},{0,203,255},{0,199,255},
{0,195,255},{0,191,255},{0,187,255},{0,183,255},{0,179,255},{0,175,255},{0,171,255},{0,167,255},{0,163,255},{0,159,255},
{0,155,255},{0,151,255},{0,147,255},{0,143,255},{0,139,255},{0,135,255},{0,131,255},{0,128,255},{0,124,255},{0,120,255},
{0,116,255},{0,112,255},{0,108,255},{0,104,255},{0,100,255},{0,96,255},{0,92,255},{0,88,255},{0,84,255},{0,80,255},
{0,76,255},{0,72,255},{0,68,255},{0,64,255},{0,60,255},{0,56,255},{0,52,255},{0,48,255},{0,44,255},{0,40,255},{0,36,255},
{0,32,255},{0,28,255},{0,24,255},{0,20,255},{0,16,255},{0,12,255},{0,8,255},{0,4,255},{0,0,255},{0,0,251},{0,0,247},
{0,0,243},{0,0,239},{0,0,235},{0,0,231},{0,0,227},{0,0,223},{0,0,219},{0,0,215},{0,0,211},{0,0,207},{0,0,203},{0,0,199},
{0,0,195},{0,0,191},{0,0,187},{0,0,183},{0,0,179},{0,0,175},{0,0,171},{0,0,167},{0,0,163},{0,0,159},{0,0,155},{0,0,151},
{0,0,147},{0,0,143},{0,0,139},{0,0,135},{0,0,131},{0,0,128} };

//! uToCrack() calculates crack opening or sliding according to given displacement fields.
/*!
\details This function calculates crack opening or sliding according to given displacement fields.
The displacement fields are normally analyzed by template match, ECC, optical flow, or other methods.
\param u displacement field. Type:CV_32FC2 in unit of pixel. Right-ward/down-ward positive. (image coordinate)
\param crack_opn crack opening field. Type:CV_32F, in unit of pixel. 
\param crack_sld crack sliding field. Type:CV_32F, in unit of pixel. 
\param angle assumed crack direction (0, 45, 90, 135 for specific direction, or 999 for max. of all above)
\param oper if oper (operator) is 0, crack_opn/sld = calculated value. 
            if oper is 1 and crack_opn/sld is allocated in corrected sizes, crack_opn/sld = max(crack_opn/sld, calculated values) 
\return 0.
*/
int uToCrack(const cv::Mat & u, cv::Mat & crack_opn, cv::Mat & crack_sld,
	int angle = 999, int opr = 0); 

//! uToStrain() calculates strain fields according to given displacement fields.
/*!
\details This function calculates strain fields according to given displacement fields.
The displacement fields are normally analyzed by template match, ECC, optical flow, or other methods.
\param u displacement field. Type:CV_32FC2 in unit of pixel. Right-ward/down-ward positive. (image coordinate)
\param exx strain field exx. Type:CV_32F, dimensionless.
\param exx strain field eyy. Type:CV_32F, dimensionless.
\param exx strain field exy. Type:CV_32F, dimensionless.
\return 0.
*/
int uToStrain(const cv::Mat & u, cv::Mat & exx, cv::Mat & eyy, cv::Mat & exy);

cv::Mat sobel_xy(const cv::Mat & src);

void imshow_resize(std::string winname, cv::Mat img, double factor); 
void imshow_resize(std::string winname, cv::Mat img, cv::Size maxWinSize);

//! tryOpcvCapFocusExposureGain() allows user to try the best focus/exposure/gain values
// by trial and error interactively (thruogh keyboard)
/*!
\param result adjusted value by user. [0]:cam_id, [1]:focus, [2]:exposure
\return 0.
*/
int tryOpcvCapFocusExposure(std::vector<double> & result); 

// Example:
// Input: vector<double>{1., 6., 2., 6. ,4., 3.};
// Return: vector<int>{0, 2, 5, 4, 1, 3};
//
template <typename T>
std::vector<size_t> sort_index(const std::vector<T> &v);


//
// This function is designed for brute-force trial-and-error for an n-dimensional problem.
//
// Example:
// Input:
//     nDim = 3,
//     nOptions = {2, 3, 2}
//     composition = {0, 0, 0}
// Output: (index) 0
// More details of the example
// index | composition when nDim = 3
//       |  and nOptions = {4, 3, 2}
// --------------------
//    0  | 0   0   0
//    1  | 1   0   0
//    2  | 2   0   0
//    3  | 3   0   0
//    4  | 0   1   0
//    5  | 1   1   0
//    6  | 2   1   0
//    7  | 3   1   0
//    8  | 0   2   0
//    9  | 1   2   0
//   10  | 2   2   0
//   11  | 3   2   0
//   12  | 0   0   1
//   13  | 1   0   1
//   14  | 2   0   1
//   15  | 3   0   1
//   16  | 0   1   1
//   17  | 1   1   1
//   18  | 2   1   1
//   19  | 3   1   1
//   20  | 0   2   1
//   21  | 1   2   1
//   22  | 2   2   1
//   23  | 3   2   1
int indexOfComposition(int nDim, const int* nOptions, const int* composition);
void compositionOfIndex(int nDim, const int* nOptions, int index, int* composition);
// The same function name but using vector<int> rather than int*.
int indexOfComposition(const std::vector<int> & nOptions, const std::vector<int> & composition);
// The same function name but using vector<int> rather than int*.
void compositionOfIndex(const std::vector<int> & nOptions, int index, std::vector<int> & composition);
double normOfVector(const std::vector<double> & x);
void printVector(std::vector<int> x);
void printVector(std::vector<double> x);

//*! \brief brute-force based solver
// * \param nIn number of input arguments of the objective function
// * \param nOut number of output argument of the objective function
// * \param func objective function with arguments: (const std::vector<double> & x, std::vector<double> & y, const void * misc)
// * \param xlowerBounds lower bounds of x arguments (must be in size: nIn)
// * \param xupperBounds upper bounds of x arguments (must be in size: nIn)
// * \param nTrialx number of trials between the lower bound and upper bound of each argument (must be in size: nIn)
// * \param misc an additional pointer to pass to the objective function. Can be NULL if nothing is to pass to the objective func.
// * \param nBest number of demanded best trial solutions
// * \param optxs best solutions x, sized nBest. Each is a solution x.
// * \param optys corresponding y of best solutions
// * \param debug for debugging (by printing some info.)
// */
// Example:
//   int objFunc(const vector<double> & x, vector<double> & y, const void * misc = NULL)
//   {
//       y[0] = 1.00 * x[0] + 0.05 * x[1] + 0.07 * x[2] - 3.0;
//       y[1] = 0.05 * x[0] + 1.00 * x[1] + 0.03 * x[2] - 5.0;
//       y[2] = 0.07 * x[0] + 0.03 * x[1] + 1.00 * x[2] - 8.0;
//       return 0;
//   }
//   int main()
//   {
//      int nIn = 3;
//      int nOut = 3;
//      int nBest = 5;
//      void * misc = NULL;
//      vector<double> xlowerBounds{0, 0, 0};
//      vector<double> xupperBounds{10, 10, 10};
//      vector<int> nTrialx{101, 101, 101};
//      vector<double> optx, opty;
//      vector<vector<double>> optxs, optys;
//      bruteForceOneLevel(nIn, nOut, objFunc,
//                         xlowerBounds, xupperBounds, nTrialx,
//                         (const void*) misc,
//                         nBest,
//                         optxs, optys);
//      for (int i = 0; i < nBest; i++)
//      {
//          printVector(optxs[i]); cout << " "; printVector(optys[i]);
//          cout << " Norm: " << normOfVector(optys[i]) << "\n";
//      }
//    }
//    Output would be:
//         2.200e+00   4.700e+00   7.700e+00   -2.600e-02   4.100e-02  -5.000e-03  Norm: 0.0488057
//         2.200e+00   4.600e+00   7.700e+00   -3.100e-02  -5.900e-02  -8.000e-03  Norm: 0.0671267
//         2.300e+00   4.700e+00   7.700e+00    7.400e-02   4.600e-02   2.000e-03  Norm: 0.087155
//         2.300e+00   4.600e+00   7.700e+00    6.900e-02  -5.400e-02  -1.000e-03  Norm: 0.0876242
//         2.200e+00   4.700e+00   7.800e+00   -1.900e-02   4.400e-02   9.500e-02  Norm: 0.106405

int bruteForceOneLevel(int nIn, int nOut, int (*func)(const std::vector<double> &, std::vector<double> &, const void *),
                       const std::vector<double> xlowerBounds, const std::vector<double> xupperBounds,
                       const std::vector<int> nTrialx,
                       const void* misc,
                       int nBest,
                       std::vector<std::vector<double>> & optxs, std::vector<std::vector<double>> & optys, int debug = 0);

/*! \brief q4 membrane strain fields
/ * \param x0 q4 nodal coordinates before deformation (zero strain) (cv::Mat(4, 3, CV_32/64F), cv::Mat(4, 1, CV_32/64FC3), cv::Mat(1, 4, CV_32/64FC3)). Nodal ordering: lower-left, lower-right, upper-right, upper-left (counter-clockwise, horizontal first)
// * \param x1 q4 nodal coordinates after deformation (cv::Mat(4, 2, CV_32/64F), cv::Mat(4, 1, CV_32/64FC3), cv::Mat(1, 4, CV_32/64FC3))
// * \param strains (cv::Mat(5, 7, CV_64F)), [epxx epyy gammaxy e1 e2 gammas ev] at the middle point of element and each of four Gauss integ. points. epxx and epyy are epsilon xx and yy; gammaxy is engineering shear strain xy; e1 and e2 are max/min principal strains, es is max. shear strain. ev is equivl. von Mises strain. Format: cv::Mat(4, 7, CV_32/64F)
// * \param poisson poisson ratio. By assuming plane stress, the out-of-plane strain is estimated by principal strain e1, e2, and poisson ratio. 
// * \return 0:success. 
*/
int q4MembraneStrains(const cv::Mat& x0, const cv::Mat& x1, cv::Mat& strains, double poisson = 0.3);

