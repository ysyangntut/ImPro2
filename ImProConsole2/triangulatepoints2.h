#ifndef TRIANGULATEPOINTS2_H
#define TRIANGULATEPOINTS2_H

#include "opencv2/core/core.hpp"
#include <vector>

int triangulatePoints2(
  const cv::Mat & camMatrix1,  // 3x3 camera matrix of cam 1
  const cv::Mat & distVect1,   // 4x1 or 5x1 distortion coefficients
  const cv::Mat & camMatrix2,  // 3x3 camera matrix of cam 2
  const cv::Mat & distVect2,   // 4x1 or 5x1 distortion coefficients
  const cv::Mat & rotation,    // 3x3 Rotation matrix between the coordinate systems of the first and the second cameras.
  const cv::Mat & tvec,        // 3x1 or 1x3 Translation vector between coordinate systems of the cameras.
  const cv::Mat & points1,     // 2xN or 2-channel 1xN or Nx1 image points on photo taken by camera 1
  const cv::Mat & points2,     // 2xN or 2-channel 1xN or Nx1 image points on photo taken by camera 2
        cv::Mat & points3d1,   // 1xN 3-channel triangulated based on cam-1 coordinate
        cv::Mat & points3d2,   // 1xN 3-channel triangulated based on cam-2 coordinate
        cv::Mat & err       )  // 1xN 1-channel, norm of error (between points and back-projected points) of each point
;

int triangulatePoints2_vecs(
	const cv::Mat & camMatrix1,  // 3x3 camera matrix of cam 1
	const cv::Mat & distVect1,   // 4x1 or 5x1 distortion coefficients
	const cv::Mat & camMatrix2,  // 3x3 camera matrix of cam 2
	const cv::Mat & distVect2,   // 4x1 or 5x1 distortion coefficients
	const cv::Mat & rotation,    // 3x3 Rotation matrix between the coordinate systems of the first and the second cameras.
	const cv::Mat & tvec,        // 3x1 or 1x3 Translation vector between coordinate systems of the cameras.
	const std::vector<cv::Point2f> & points1,     // image points on photo taken by camera 1
	const std::vector<cv::Point2f> & points2,     // image points on photo taken by camera 2
	std::vector<cv::Point3d> & points3d1,         // triangulated points based on cam-1 coordinate
	std::vector<cv::Point3d> & points3d2,         // triangulated points based on cam-2 coordinate
	std::vector<double> & err);  // norm of error (between points and back-projected points) of each point

int triangulatePointsGlobalCoordinate(
	const cv::Mat & camMat1,
	const cv::Mat & disVec1,
	const cv::Mat & r4Mat1,
	const cv::Mat & camMat2,
	const cv::Mat & disVec2,
	const cv::Mat & r4Mat2,
	const cv::Mat & points1,     // 2xN or 2-channel 1xN or Nx1 image points on photo taken by camera 1
	const cv::Mat & points2,     // 2xN or 2-channel 1xN or Nx1 image points on photo taken by camera 2
	cv::Mat & points3d,
	cv::Mat & err
); 

int triangulatePointsGlobalCoordinate(
	const cv::Mat & camMat1,
	const cv::Mat & disVec1,
	const cv::Mat & rmat1,
	const cv::Mat & tvec1, 
	const cv::Mat & camMat2,
	const cv::Mat & disVec2,
	const cv::Mat & rmat2,
	const cv::Mat & tvec2,
	const cv::Mat & points1,     // 2xN or 2-channel 1xN or Nx1 image points on photo taken by camera 1
	const cv::Mat & points2,     // 2xN or 2-channel 1xN or Nx1 image points on photo taken by camera 2
	cv::Mat & points3d,
	cv::Mat & err
);


// {
// 	cv::Mat points1Mat(1, points1.size(), CV_32FC2);
// 	cv::Mat points2Mat(1, points2.size(), CV_32FC2);
// 	cv::Mat points3d1Mat(1, points1.size(), CV_64FC3);
// 	cv::Mat points3d2Mat(1, points1.size(), CV_64FC3);
// 	cv::Mat errMat(1, points1.size(), CV_64F);
// 	// copy vector<Point2f> to Mat(1,n,CV_32FC2)
// 	for (int i = 0; i < points1.size(); i++) points1Mat.at<cv::Point2f>(i) = points1[i];
// 	for (int i = 0; i < points2.size(); i++) points2Mat.at<cv::Point2f>(i) = points2[i];
// 	int ret = triangulatePoints2(camMatrix1, distVect1, camMatrix2, distVect2, rotation, tvec,
// 		points1Mat, points2Mat,
// 		points3d1Mat, points3d2Mat, errMat);
// 	// copy Mat(1,n,CV_64FC3) to vector<Point3d>
// 	for (int i = 0; i < points1.size(); i++) points3d1[i] = points3d1Mat.at<cv::Point3d>(i);
// 	for (int i = 0; i < points2.size(); i++) points3d2[i] = points3d2Mat.at<cv::Point3d>(i);
// 	for (int i = 0; i < points2.size(); i++) err[i] = errMat.at<double>(i);
// 	return ret;
// }

#endif // TRIANGULATEPOINTS2_H

