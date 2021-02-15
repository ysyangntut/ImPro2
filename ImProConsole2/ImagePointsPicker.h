#pragma once
#include <string>
#include <opencv2/core.hpp>

using namespace std;
using namespace cv;

//! ImagePointsPicker picks points by mouse on an image. 
/*!
ImagePointsPicker provides opens an window showing an image and 
allows user to pick points on the image. The image coordinates 
are stored as a cv::Mat and can be returned through Nx1 cv::Mat, 
 or vector<cv::Point2f>. 
*/

class ImagePointsPicker
{
public:
	//! Constructs an ImagePointsPicker
	ImagePointsPicker();
	~ImagePointsPicker();

	//! Sets background image by giving a file name 
	int setBackgroundImageByFilename(const std::string filename);
//	int setBackgroundImageByMat(const cv::Mat & img);

	//! Requests user to pick points by mouse.
	/*!
	\details Points data will be saved at this->points (vector of Points2f) . 
	Point data are stored at this->points (vector of Points2f) 
	If user skips a point by ESC (bKey), the point becomes cv::Point2f(nanf(), nanf())
	\param nPoint number of points to pick
	\param bKey break key (by default ESC)
	\param cKey1 confirm key # 1 (by default 0, meaning no confirmation waiting)
	\param cKey2 confirm key # 2 (by default 0, meaning no confirmation waiting)
	\return number of picked points (supposed to be nPoint minus ESC skipped point), not including
	skipped points. Something wrong if it returns a negative value.
	*/
	int pickPoints(int nPoint, int bKey = 27, int cKey1 = 0, int cKey2 = 0); 

	//! Requests user to pick points and ROIs by mouse.
	/*!
	\details Points and ROIs data will be saved at this->points (vector of Points2f) and this->rois (vector of Rect).
	Point data are stored at at this->points (vector of Points2f) and this->rois (vector of Rect).
	If user skips a point by ESC (bKey), the point becomes cv::Point2f(nanf(), nanf()), and the ROI becomes cv::Rect(0,0,0,0)
	\param nPoint number of points with ROIs to pick
	\param bKey break key (by default ESC)
	\param cKey1 confirm key # 1 (by default 0, meaning no confirmation waiting)
	\param cKey2 confirm key # 2 (by default 0, meaning no confirmation waiting)
	\return number of picked points with ROIs (supposed to be nPoint minus ESC skipped point), not including
	skipped points. Something wrong if it returns a negative value.
	*/
	int pickTemplates(int nPoint, int bKey = 27, int cKey1 = 0, int cKey2 = 0);

	//! Requests user to pick four points by mouse and generate q4 mesh for ROIs.
	/*!
	\details Points and ROIs data will be saved at this->points (vector of Points2f) and this->rois (vector of Rect).
	Four corners can be either clockwise or counter clockwise.  
	Point data are stored at at this->points (vector of Points2f) and this->rois (vector of Rect).
	If user skips a point by ESC (bKey), the point becomes cv::Point2f(nanf(), nanf()), and the ROI becomes cv::Rect(0,0,0,0)
	\param nPoint12 number of points along points 1 and 2 with ROIs to pick
	\param nPoint23 number of points along points 2 and 3 with ROIs to pick
	\param bKey break key (by default ESC)
	\param cKey1 confirm key # 1 (by default 0, meaning no confirmation waiting)
	\param cKey2 confirm key # 2 (by default 0, meaning no confirmation waiting)
	\return number of picked points with ROIs (supposed to be nPoint minus ESC skipped point), not including
	skipped points. Something wrong if it returns a negative value.
	*/
	int pickQ4Templates(int nPoint12, int nPoint23, int bKey = 27, int cKey1 = 0, int cKey2 = 0);

	//! If there are 4 defined points, this function converts 4-point data to a Q4 mesh points data.
	/*!
	\details This function only works where number of points is 4, and must be 4. If not, this
	function does not do anything. If it works, the data will be replaced with refined mesh points.
	\param nPoint12 number of points along points 1 and 2 with ROIs to pick
	\param nPoint23 number of points along points 2 and 3 with ROIs to pick
	*/
	int generateQ4Templates(int nPoint12, int nPoint23);


	//! Returns number of points (size of this->points)
	/*!
	\return number of points (size of this->points)
	*/
	int num_points(); 

	//! Returns the points in vector<cv::Point2f> format
	vector<cv::Point2f> pointsInPoint2fVector() const; 

	//! Returns the points in 1xN cv::Mat (CV_32FC2) 
	cv::Mat pointsIn1xNCvMat() const;

	//! Returns the cv::Rect in vector<cv::Rect> format. Each Rect represents the area of a Point2f. 
	vector<cv::Rect> rectsInRectVector() const;

	//! Writes an image of the background image with marked points.
	/*!
	\param fname if empty, output file will be [dir/file]_marked.JPG. 
	             if fname has no directory, output file will be 
				 the same as the background directory [dir]
				 if relative or full directory is given, output will be there.
 	*/
	int writeMarkersToImageFile(std::string fname = std::string(""), int draw_type = 2, int mark_size = 10);

	//! Writes an image of the background image with marked templates.
	/*!
	\param fname if empty, output file will be [dir/file]_marked.JPG.
				 if fname has no directory, output file will be
				 the same as the background directory [dir]
				 if relative or full directory is given, output will be there.
	*/
	int writeTemplateMarkersToImageFile(std::string fname = std::string(""), int draw_type = 2, int mark_size = 10);


	//! Writes points into text file (x1 y1\nx2 y2\nx3 y3\n...\nxn yn\n)
	/*!
	\param fname if empty, output file will be [dir/file]_pickedPoints.txt.
				 if fname has no directory, output file will be
				 the same as the background directory [dir]
				 if relative or full directory is given, output will be there.
	\return 0 for success. -1 for something wrong.
	*/
	int writePointsToTxtFile(std::string fname = std::string(""));

	//! Writes templates into text file (x1 y1 rect1.x rect1.y rect1.width rect1.height \n x2 y2 rect2.x ... \n xn yn rectn.x ... \n)
	/*!
	\param fname if empty, output file will be [dir/file]_pickedTemplates.txt.
				 if fname has no directory, output file will be
				 the same as the background directory [dir]
				 if relative or full directory is given, output will be there.
	\return 0 for success. -1 for something wrong.
	*/
	int writeTemplatesToTxtFile(std::string fname = std::string(""));

	//! Writes points through OpenCV FileStorage
	/*!
	\param fname if empty, output file will be [dir/file]_pickedPoints.xml.
				 if fname has no directory, output file will be
				 the same as the background directory [dir]
				 if relative or full directory is given, output will be there.
	\return 0 for success. -1 for something wrong.
	*/
	int writePointsToXmlFile(std::string fname = std::string(""));

	//! Writes templates through OpenCV FileStorage
	/*!
	\param fname if empty, output file will be [dir/file]_pickedTemplates.xml.
				 if fname has no directory, output file will be
				 the same as the background directory [dir]
				 if relative or full directory is given, output will be there.
	\return 0 for success. -1 for something wrong.
	*/
	int writeTemplatesToXmlFile(std::string fname = std::string(""));

	//! Checks image with marked points exists or not.
	/*!
	\return true if file exists, false if not.
	*/
	bool existMarkersImageFile();

	//! Checks points of text file exists or not
	/*!
	\return true if file exists, false if not. 
	*/
	bool existPointsTxtFile(std::string fname = std::string(""));

	//! Checks points of OpenCV FileStorage exists or not
	/*!
	\param fname Given file name. If empty, use default filename. 
	 If only file name without directory, the directory is the same as background file.
	\return true if file exists, false if not.
	*/
	int existPointsXmlFile(std::string fname = std::string(""));

	//! Reads points from text file (x1 y1\nx2 y2\nx3 y3\n...\nxn yn\n)
	/*!
	\param fname Given file name. If empty, use default filename.
	 If only file name without directory, the directory is the same as background file.
	\return 0 for success. -1 for something wrong.
	*/
	int readPointsFromTxtFile(std::string fname = std::string(""));

	//! Writes points through OpenCV FileStorage
	/*!
	\param fname Given file name. If empty, use default filename.
	 If only file name without directory, the directory is the same as background file.
	\return 0 for success. -1 for something wrong.
	*/
	int readPointsFromXmlFile(std::string fname = std::string(""));

	//! Refreshes marked image 
	/*!
	\details refreshMarkedImage() reads background photo from file and refreshes 
	the marked image to background photo. 
	\return 0 for success. -1 for something wrong.
	*/
	int refreshMarkedImage();

	//! Paint picked point number on marked image
	/*!
	\details paintMarkerOnPoint() paints a marker and its number on the background image
	\param iPoint 0-based point id
	\param draw_type marker type. 1:circle, 2:cross
	\param mark_size the size of the marker to draw. 
	\return 0 for success. otherwise for something wrong.
	*/
	int paintMarkerOnPoint(int iPoint, int draw_type = 2, int mark_size = 10);

	//! Paint picked template number on marked image
	/*!
	\details paintTemplateMarkerOnPoint() paints a marker and its number on the background image
	\param iPoint 0-based point id
	\param draw_type marker type. 1:circle, 2:cross
	\param mark_size the size of the marker to draw.
	\return 0 for success. otherwise for something wrong.
	*/
	int paintTemplateMarkerOnPoint(int iPoint, int draw_type = 2, int mark_size = 10);

	/*!
	\return image size. 
	*/
	cv::Size imgSize();

private:
	int nPoint; 
	std::string imgfile;
	std::string imgfile_marked;
	std::string txtfile, xmlfile;

	cv::Mat img; 
	cv::Mat img_marked;
	std::vector<cv::Point2f> points;
	std::vector<cv::Rect> tmpltRects; 
};

