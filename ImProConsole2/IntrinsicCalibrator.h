#pragma once
#include <vector>
#include <iostream>
using namespace std;

#include <opencv2/core.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
using namespace cv;

#include "FileSeq.h"
//#include "FileSequence.h"

#define ICAL_CORNERS_FOUND 1
#define ICAL_CORNERS_FAILED -1
#define ICAL_CORNERS_UNKNOWN 0

vector<double> proj_err_points(
	const vector<cv::Point2f> imgPoints,
	const vector<cv::Point3f> objPoints,
	const cv::Mat cmat, const cv::Mat dvec,
	const cv::Mat rvec, const cv::Mat tvec);

vector<double> proj_err_lines(
	const vector<cv::Point2f> imgPoints,
	const cv::Mat cmat, const cv::Mat dvec);

/*! 
IntrinsicCalibrator assists the procedures to carry out intrinsic 
calibration. It needs user to provide a text file that contains 
calibration coordinate of corners on a calibration board. 

Usage: 
  // Step 1: Create an  IntrinsicCalibrator
  IntrinsicCalibrator calib;
  // 
  Step 2: Set path of files
  Step 3: You can get all file names in this path 

 calib.readFromFsFile("calibration_myCamera.xml/yaml");
 calib.findAllCorners();
 calib.writeToFsFile("calibration_myCamera_result.xml/yaml");

*/

class IntrinsicCalibrator
{
public:
	IntrinsicCalibrator();
	~IntrinsicCalibrator();

	//! Constructs an IntrinsicCalibrator object by giving working directory
	/*!
	\details
	Usage example:
	IntrinsicCalibrator calib("c:/TestPhotos/");
	\param theDir the directory of calibration photos. Files must be
	in the working directory.
	The directory separator can be either '/' or '\\'.
	If theDir does not end with '/' or '\\' then this class
	will automatically add one.
	*/
	IntrinsicCalibrator(std::string theDir);

	//! Initialization of an IntrinsicCalibrator object
	/*!
	*/
	void init(); 

	//! Returns the reference of FileSeq object
	FileSeq & fileSeq(); 

	// createTemplateInputFile() writes the XML file of an empty
	// IntrinsicCalibrator by calling writeToFsFile() giving xml and yaml file names
	// of a new constructed object.
	void createTemplateInputFile();

	// setCalibrationBoard() sets chessboard-pattern calibration board parameters
	//  including number of corners along width and height directions, and the 
	//  physical size between two neighboring corners along width and height 
	//  directions.  For a regular 8x8 chessboard with 2-inch squares, give 
	//  (7, 7, 2, 2) if you use inch, or (7, 7, 50.8, 50.8) if you use mm.   
	void setCalibrationBoard(int type, int num_corners_along_width, int num_corners_along_height,
		double square_width, double square_height);

	// setFileSequence() allows user to set calibration photos through an
	// FileSequence.
	// Input:
	//		imsq: the FileSequence of calibration photos
	// Return:
	//      number of valid photos
	int setFileSeq(const FileSeq & imsq);

	// addCalibPhoto() allows user to add a single calibration photo. 
	// Input: 
	//		img: calibration photo
	//      bsize_w_h: numbers of squares along board width (.x) and height (.y)
	//      square_w: the physical size of the width of a square (user defines the unit)
	//      square_h: the physical size of the height of a square (user defines the unit)
	// Return:
	//      1: success
	//      0 or else: cannot find sufficient corners 
	int addCalibrationPhoto(const cv::Mat & img, const cv::Size bsize_w_h, 
		double square_w, double square_h);

	//! findCorners() tries to the calibration board corners in a photo.
	/*!
	\param idx index of photo in the file sequence
	\param bSize board size (number of points along width and height)
	\param sqw square size along width
	\param sqh square size along height 
	\param board_type board type. 1:chessboard, 2.grid(sym), 3.grid(unsym)
	\return 0: success.  -1: File is not an image.  -2: Corners cannot be found.
	*/
	int findCorners(int idx, cv::Size bSize, float sqw, float sqh, 
		int board_type = 1);

	//! cal_types() returns the vector of calibration types
	vector<int> & calTypes(); 
	const vector<int> & calTypes() const;

	//! imgPoints() returns the vector of vectors of image points
	vector<vector<Point2f> > & imgPoints(); // calib_imgPoints[iimg][ipoint] is image  point fo ipoint of iimg-th calibration photo 
	const vector<vector<Point2f> > & imgPoints() const; // calib_imgPoints[iimg][ipoint] is image  point fo ipoint of iimg-th calibration photo 

	//! objPoints() returns the vector of vectors of object points
	vector<vector<Point3f> > & objPoints(); // calib_objPoints[iimg][ipoint] is object point of ipoint of iimg-th calibration photo 
	const vector<vector<Point3f> > & objPoints() const; // calib_objPoints[iimg][ipoint] is object point of ipoint of iimg-th calibration photo 

	//! writeImgPointsToFile() writes image points to both .xi.txt, .xi.xml, and .xi.yaml files.
	/*!
	\param idx index of photo in the file sequence
	*/
	int writeImgPointsToFile(int idx);

	//! writeImgsPointsToFiles() writes all images points to both .xi.txt, .xi.xml, and .xi.yaml files.
	int writeImgsPointsToFiles();

	//! writeObjPointsToFiles() writes object points to both .xxi.txt .xi.xml, and .xi.yaml files.
	/*!
	\param idx index of photo in the file sequence
	*/
	int writeObjPointsToFile(int idx);

	//! writeObjsPointsToFiles() writes all objects points to both .xi.txt, .xi.xml, and .xi.yaml files.
	int writeObjsPointsToFiles();

	// findAllCorners() tries to find the corners. It may take a few seconds of time.
	/*!
	\param bSize board size(number of points along width and height)
	\param sqw square size along width
	\param sqh square size along height
	\param board_type board type. 1:chessboard, 2.grid(sym), 3.grid(unsym)
	\return 0: success. - 1 : File is not an image. - 2 : Corners cannot be found.
	*/
	int findAllCorners(cv::Size bSize, float sqw, float sqh,
		int board_type = 1);

	//! setBoardObjPoints() sets calibration board object points of a photo
	/*!
	\param idx index of photo in the file sequence
	\param bSize board size (number of points along width and height)
	\param sqw square size along width
	\param sqh square size along height
	\param board_type board type. 1:chessboard, 2.grid(sym), 3.grid(unsym)
	\return 0: success
	*/
	int setBoardObjPoints(int idx, cv::Size bSize, float sqw, float sqh,
		int board_type = 1);

	//! defineUserPoints
	/*!
	\details user defines image points by mouse and object points by keyboard
	\param idx photo index
	\param imgPoints in type of vector<Point2f>
	\param objPoints in type of vector<Point3f>
	*/
	int defineUserPoints(int idx,
		const vector<cv::Point2f> & imgPoints,
		const vector<cv::Point3f> & objPoints, 
		cv::Size imgSize = cv::Size(0,0) ); 
	int defineUserPoints(int idx,
		const vector<cv::Point2f> & imgPoints,
		const vector<cv::Point3d> & objPoints,
		cv::Size imgSize = cv::Size(0, 0));

	int setCameraMatrix(const cv::Mat & cmat);
	int setDistortionVector(const cv::Mat & dvec);
	int solvePnp(int valid_photo_id = 0); 

	//! get image points of photo idx
	/*!
	\details returns image points of photo idx
	\param idx photo index
	*/
	const vector<cv::Point2f> & imagePoints(int idx); 

	//! get object points of photo idx
	/*!
	\details returns object points of photo idx
	\param idx photo index
	*/
	const vector<cv::Point3f> & objectPoints(int idx);


	// set flags
	int setFlag_FixAspectRatio(bool flag); 
	int setFlag_FixFocalLength(bool flag);
	int setFlag_FixK1(bool flag);
	int setFlag_FixK2(bool flag);
	int setFlag_FixK3(bool flag);
	int setFlag_FixK4(bool flag);
	int setFlag_FixK5(bool flag);
	int setFlag_FixK6(bool flag);
	int setFlag_FixPrincipalPoint(bool flag); 
	int setFlag_RationalModel(bool flag);
	int setFlag_FixTangentDist(bool flag);
	int setFlagByAsking(); 
	
	// 

	//! calibrate() runs intrinsic calibration according to all 
	// corners-found calibration photos. 
	/*!
	\details calibrate() runs opencv intrinsic calibration after checking data. 
	   Only points sets with the following conditions are selected for calibration
	   - Calibration type (cal_types) is 1, 2, 3 (boards) or 11 (user defined)
	   - Correct numbers (>0, # img points = # obj points) of image and object points
	   - All points are valid (not isnan)
	   - If invalid, cal_types will be changed to 0 (not assigned)
	   After calibration, the following info are saved
	   - camera matrix (cmat) / distortion vector (dvec)
	   - rves and tvecs
	   .... 
	 \param flagType: 0: (default) by user's previous setting (this->calib_flag). 1: according to number of points. 2: given in 2nd argument
	 \param flag: calibration flag, only used when flagType == 2
	 */
	int calibrate(int flagType = 0, int flag = 0); 

	//! calibrateWithGivenLevel() runs intrinsic calibration considering user assigned level
	/*!
	 \param level flag level: level 1:fx(=fy),k1, 2:fx,fy,k1, 3:fx,fy,cx,cy,k1, 4:fx,fy,cx,cy,k1,p1,p2, 5:fx,fy,cx,cy,k1,k2,p1,p2.
	 */
	int calibrateByLevel(int level);


	// numValidPhotos() returns the number of valid photos.
	// Valid photos are which all corners on calibration board can be found. 
	// Return:
	//      The number of valid photos.
	int numValidPhotos() const;

	// setImageSize() sets the image size. 
	// If given non-positive values, this function tries to find size from file or 
	// user defined points.
	int setImageSize(int w = 0, int h = 0);

	// imageSize() returns the image size. It updates when photos are read and corners 
	//      are found, or data are loaded from file.  
	//      It becomes zero when setFileSequence() is called.  
	cv::Size imageSize() const;

	// validCalibrationFileName() returns the file name of the valid photo. 
	// Valid photos are which all corners on calibration board can be found. 
	// Input: 
	//      index: the index of the valid calibration photo
	// Return:
	//      The file name of the valid photo.
	string validCalibrationFileName(int index) const; 

	// validCalibrationRms() returns root mean square of errors of the 
	// valid photo. 
	// Valid photos are which all corners on calibration board can be found. 
	// Input: 
	//      index: the index of the valid calibration photo
	// Return:
	//      The root mean square of errors of the valid photo.
	double validCalibrationRms(int index) const;

	// cameraMatrix() returns the calibrated camera matrix
	// Return:
	//     The calibrated camera matrix. If calibration is not done yet, it 
	//     returns eye(3,3, CV_32F).
	cv::Mat cameraMatrix(); // camera matrix format cv::Mat(3, 3, CV_64F)

	// distortionVector() returns the calibrated distortion vector
	// Return:
	//     The calibrated distortion vector. If calibration is not done yet, it returns eye(3,3, CV_32F).
	cv::Mat distortionVector();  // distortion coefficients format cv::Mat(1, DN, CV_64F), DN = 4,5,8,12, 

	//! rotationVectors() returns vector of the r-vec of each photo
	vector<Vec3d> rotationVectors(); 

	//! translationVectors() returns vector of the t-vec of each photo
	vector<Vec3d> translationVectors();

	//! projection_points_vecvec() calculates projected points 
	// of each point of each photo
	/*!
	\details Projection points are in unit of "pixel."
	\return vector<vector<double> > 
	*/
	vector<vector<cv::Point2f> > projection_points_vecvec();

	//! projection_errors_vecvec() calculates projection error
	// of each point of each photo
	/*!
	\details Projection errors are in unit of "pixel." 
	\return vector<vector<double> > proj_errs 
	*/
	vector<vector<double> > projection_errors_vecvec(); 

	//! projection_errors_vec() calculates projection error
	// of each photo (norm of each point) 
	/*!
	\details Projection errors are in unit of "pixel."
	\return vector<double> proj_errs
	*/
	vector<double> projection_errors_vec();

	//! projection_error() calculates projection error
	// overall (norm of norm of all points)  
	/*!
	\details Projection error is in unit of "pixel."
	\return norm of each photo of each point
	*/
	double projection_error();

	// writeIntrinsicParametersToFile() writes intrinsic parameters to a file. 
	// Output parameters are fx,fy,cx,cy,k1,k2,p1,p2,k3,k4,k5,k6
	// Input: 
	//     filename: the file name to write (without the path). The file will be written at the same
	//               directory with the FileSequence. 
	//               File name of "cout" is a special string that indicates the cout stream.
	//               However, if FileSequence is not assigned yet, the filename should be a full path. 
	// Return:
	//     0: succeeded 
	//     otherwise: failed
	int writeIntrinsicParamteresToFile(string filename = "cout") const; 


	// readIntrinsicParametersToFile() reads intrinsic parameters from a file. 
	// Input parameters are fx,fy,cx,cy,k1,k2,p1,p2,k3,k4,k5,k6
	// Input: 
	//     fullpathfile: the file name to read (with full path). 
	// Return:
	//     0: succeeded 
	//     otherwise: failed
	int readIntrinsicParamteresToFile(string fullpathfile) ;

	// writeToFsFile() writes settings to an XML/YAML file.
	int writeToFsFile(string fullPathSetting) const; 

	// readFromFsFile() writes settings to an XML file.
	int readFromFsFile(string fullPathSetting);

	//! Returns R4 matrix of a certain photo 
	/*!
	*/
	cv::Mat R4(int i) const;


	// get FileSequence
	FileSeq getFileSeq() const;

	//! Writes string to log file
	/*!
	*/
	int log(std::string) const;

	//! Writes a matlab script for visualization
	//! Runs a sample program
	/*!
	*/
	int writeToMscript(std::string) const;

private:
	FileSeq imsq; // File sequence of calibration photos 
	int n_calib_imgs;    // number of valid calibration photos (images) 
	int calib_flag;
	vector<int> cal_types; // 0:not assigned. 1:chessboard. 2:grid(sym). 3:grid(unsym). 11:user defined points. 12:3-point straight lines
	double calib_rms;
	vector<vector<double> > projection_errs; // unit: pixel
	cv::Mat cmat; // cmat is the calibrated camera matrix (3x3).
	cv::Mat dvec; // dvec is the calibrated distortion coefficients. (1x4, 1x5, 1x8, or 1x12)
	vector<cv::Vec3d> rvecs; // r-vec of each photo. (only valid for cal_types[i] == 1, 2, 3, 11)
	vector<cv::Vec3d> tvecs; // t-vec of each photo. (only valid for cal_types[i] == 1, 2, 3, 11)

	vector<int> findingCornersResult;  // result of finding corners
	vector<double> calib_rmsv; // Root-mean-square vector of calibration photos
	vector<vector<Point3f> > calib_objPoints; // calib_objPoints[iimg][ipoint] is object point of ipoint of iimg-th calibration photo 
	vector<vector<Point2f> > calib_imgPoints; // calib_imgPoints[iimg][ipoint] is image  point fo ipoint of iimg-th calibration photo 
	vector<vector<Point2f> > calib_prjPoints; // calib_prjPoints[iimg][ipoint] is image project point of ipoint of iimg-th calibration photo 
	vector<cv::Mat> rmats44; // extrinsic 4x4 matrix of the calibration board
	cv::Size imgSize; // image size of this camera. 

	vector<string> calib_valid_fnames; // file names of valid calibration photos. 
	vector<double> calib_valid_rms;    // rms of valid calibration photos.
	

	std::string logFilename;
};




