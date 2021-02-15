#include <iostream>
using namespace std;

#include "opencv2/core/core.hpp"
#include "opencv2/calib3d.hpp"
#include "opencv2/imgproc.hpp"
#include "triangulatepoints2.h"

void convert_to_two_channel(const cv::Mat & points, cv::Mat & xC2);

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
{
    // check dimension & make convert all to 1-channel

    // declarations
    int nPoints1 = 1, nPoints2 = 1, nPoints = 1;
    cv::Mat xLC2(1, nPoints, CV_64FC2);  // original image points in 2-channel
    cv::Mat xRC2(1, nPoints, CV_64FC2);  // original image points in 2-channel
    cv::Mat uL(2, nPoints, CV_64F), uR(2, nPoints, CV_64F);  // undistorted image points
    cv::Mat hL(3, nPoints, CV_64F), hR(3, nPoints, CV_64F);  // homogeneous image points
    cv::Mat pmL(3, 4, CV_64F), pmR(3, 4, CV_64F);            // projection matrix
    cv::Mat rcL(3, 3, CV_64F), rcR(3, 3, CV_64F);            // rectification transformation
    cv::Mat Q(4,4, CV_64F);                                  // disperity-to-depth matrix
    cv::Mat XL4, XR4;                                        // 3-d points in L/R coord.

    // convert points1 and points2 to 1xN two-channel matrices
    // because undistortPoints() needs them to be 2-channel

    convert_to_two_channel(points1, xLC2);
    convert_to_two_channel(points2, xRC2);
    nPoints1 = xLC2.cols;
    nPoints2 = xRC2.cols;
    nPoints = (nPoints1 < nPoints2) ? nPoints1 : nPoints2; // nPoints = min(nPoints1,nPoints2)
	points3d1 = cv::Mat::zeros(1, nPoints, CV_64FC3); 
	points3d2 = cv::Mat::zeros(1, nPoints, CV_64FC3);

    // find projection matrices and rectification transform matrix (for undistortion)

    stereoRectify(camMatrix1, distVect1, camMatrix2, distVect2,
                      cv::Size(1,1), rotation, tvec, rcL, rcR, pmL, pmR, Q);

    // find undistorted points
    cv::undistortPoints(xLC2, uL, camMatrix1, distVect1, rcL, pmL);
    cv::undistortPoints(xRC2, uR, camMatrix2, distVect2, rcR, pmR);

    // triangulate points
    XL4 = cv::Mat::zeros(4, nPoints, CV_64F);
    cv::triangulatePoints(pmL, pmR, uL, uR, XL4);

    // convert points4D to 3D (3-channel)

    cv::Mat XL3(3, nPoints, CV_64F), XR3(3, nPoints, CV_64F);
    for (int iPoint = 0; iPoint < nPoints; iPoint++) {
        XL3.at<double>(0,iPoint) = XL4.at<double>(0,iPoint) / XL4.at<double>(3,iPoint);
        XL3.at<double>(1,iPoint) = XL4.at<double>(1,iPoint) / XL4.at<double>(3,iPoint);
        XL3.at<double>(2,iPoint) = XL4.at<double>(2,iPoint) / XL4.at<double>(3,iPoint);
    }
    XL3 = rcL.inv() * XL3;
    for (int iPoint = 0; iPoint < nPoints; iPoint++) {
        points3d1.at<cv::Point3d>(0,iPoint).x = XL3.at<double>(0,iPoint);
        points3d1.at<cv::Point3d>(0,iPoint).y = XL3.at<double>(1,iPoint);
        points3d1.at<cv::Point3d>(0,iPoint).z = XL3.at<double>(2,iPoint);
    }

    // calculate R (from L) (3-channel)

    XR3 = rotation * XL3 + cv::repeat(tvec, 1, nPoints);
    for (int iPoint = 0; iPoint < nPoints; iPoint++) {
        points3d2.at<cv::Point3d>(0,iPoint).x = XR3.at<double>(0,iPoint);
        points3d2.at<cv::Point3d>(0,iPoint).y = XR3.at<double>(1,iPoint);
        points3d2.at<cv::Point3d>(0,iPoint).z = XR3.at<double>(2,iPoint);
    }

    // calculate error (re-project to images)
    // back-project to image points (replace projPoints1 projPoints2)
    cv::Mat projPoints1(1, nPoints, CV_64FC2), projPoints2(1, nPoints, CV_64FC2);

    cv::projectPoints(points3d1, cv::Mat::zeros(1,3,CV_64F), cv::Mat::zeros(1,3,CV_64F),
                  camMatrix1, distVect1, projPoints1);
    cv::projectPoints(points3d2, cv::Mat::zeros(1,3,CV_64F), cv::Mat::zeros(1,3,CV_64F),
                  camMatrix2, distVect2, projPoints2);

    err = cv::Mat::zeros(1, nPoints, CV_64F);
    cv::Mat errvec1(2,1,CV_64F), errvec2(2,1,CV_64F);
    for (int iPoint = 0; iPoint < nPoints; iPoint++) {
        errvec1.at<double>(0,0) = projPoints1.at<cv::Point2d>(0,iPoint).x - xLC2.at<cv::Point2d>(0,iPoint).x;
        errvec1.at<double>(1,0) = projPoints1.at<cv::Point2d>(0,iPoint).y - xLC2.at<cv::Point2d>(0,iPoint).y;
        errvec2.at<double>(0,0) = projPoints2.at<cv::Point2d>(0,iPoint).x - xRC2.at<cv::Point2d>(0,iPoint).x;
        errvec2.at<double>(1,0) = projPoints2.at<cv::Point2d>(0,iPoint).y - xRC2.at<cv::Point2d>(0,iPoint).y;
        err.at<double>(0,iPoint) = cv::norm(errvec1) + cv::norm(errvec2);
    }

    return 0;
}



// convert points to 1xN two-channel matrices
// because undistortPoints() needs them to be 2-channel
// also force to converts to 64-bit
void convert_to_two_channel(const cv::Mat & points, cv::Mat & xC2)
{
    int N;
    // if points is 1-channel
    if (points.rows == 2 && points.channels() == 1 && points.type() == CV_32F) {
        N = points.cols;
        xC2 = cv::Mat(1, N, CV_64FC2);
        for (int i = 0; i < N; i++) {
            xC2.at<cv::Point2d>(0, i).x = (double) points.at<float>(0, i);
            xC2.at<cv::Point2d>(0, i).y = (double) points.at<float>(1, i);
        }
        return;
    }
    if (points.rows == 2 && points.channels() == 1 && points.type() == CV_64F) {
        N = points.cols;
        xC2 = cv::Mat(1, N, CV_64FC2);
        for (int i = 0; i < N; i++) {
            xC2.at<cv::Point2d>(0, i).x = points.at<double>(0, i);
            xC2.at<cv::Point2d>(0, i).y = points.at<double>(1, i);
        }
        return;
    }
    if (points.cols == 2 && points.channels() == 1 && points.type() == CV_32F) {
        N = points.rows;
        xC2 = cv::Mat(1, N, CV_64FC2);
        for (int i = 0; i < N; i++) {
            xC2.at<cv::Point2d>(0, i).x = (double) points.at<float>(i, 0);
            xC2.at<cv::Point2d>(0, i).y = (double) points.at<float>(i, 1);
        }
        return;
    }
    if (points.cols == 2 && points.channels() == 1 && points.type() == CV_64F) {
        N = points.rows;
        xC2 = cv::Mat(1, N, CV_64FC2);
        for (int i = 0; i < N; i++) {
            xC2.at<cv::Point2d>(0, i).x = points.at<double>(i, 0);
            xC2.at<cv::Point2d>(0, i).y = points.at<double>(i, 1);
        }
        return;
    }

    // if points is 2-channel
    if (points.rows == 1 && points.channels() == 2 && points.type() == CV_32FC2) {
        N = points.cols;
        xC2 = cv::Mat(1, N, CV_64FC2);
        for (int i = 0; i < N; i++) {
            xC2.at<cv::Point2d>(0, i).x = (double) points.at<cv::Point2f>(0, i).x;
            xC2.at<cv::Point2d>(0, i).y = (double) points.at<cv::Point2f>(0, i).y;
        }
        return;
    }
    if (points.rows == 1 && points.channels() == 2 && points.type() == CV_64FC2) {
        N = points.cols;
        xC2 = points.clone();
        return;
    }
    if (points.cols == 1 && points.channels() == 2 && points.type() == CV_32FC2) {
        N = points.rows;
        xC2 = cv::Mat(1, N, CV_64FC2);
        for (int i = 0; i < N; i++) {
            xC2.at<cv::Point2d>(0, i).x = (double) points.at<cv::Point2f>(i, 0).x;
            xC2.at<cv::Point2d>(0, i).y = (double) points.at<cv::Point2f>(i, 0).y;
        }
        return;
    }
    if (points.cols == 1 && points.channels() == 2 && points.type() == CV_64FC2) {
        N = points.rows;
        xC2 = cv::Mat(1, N, CV_64FC2);
        for (int i = 0; i < N; i++) {
            xC2.at<cv::Point2d>(0, i).x = points.at<cv::Point2d>(i, 0).x;
            xC2.at<cv::Point2d>(0, i).y = points.at<cv::Point2d>(i, 0).y;
        }
        return;
    }

}


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
	std::vector<double> & err)  // norm of error (between points and back-projected points) of each point
{
	cv::Mat points1Mat(1, (int) points1.size(), CV_32FC2);
	cv::Mat points2Mat(1, (int)points2.size(), CV_32FC2);
	cv::Mat points3d1Mat(1, (int)points1.size(), CV_64FC3);
	cv::Mat points3d2Mat(1, (int)points2.size(), CV_64FC3);
	cv::Mat errMat(1, (int)points1.size(), CV_64F);
	// copy vector<Point2f> to Mat(1,n,CV_32FC2)
	for (int i = 0; i < (int)points1.size(); i++) points1Mat.at<cv::Point2f>(i) = points1[i];
	for (int i = 0; i < (int)points2.size(); i++) points2Mat.at<cv::Point2f>(i) = points2[i];
	int ret = triangulatePoints2(camMatrix1, distVect1, camMatrix2, distVect2, rotation, tvec,
		points1Mat, points2Mat,
		points3d1Mat, points3d2Mat, errMat);
	// copy Mat(1,n,CV_64FC3) to vector<Point3d>
	for (int i = 0; i < (int)points1.size(); i++) points3d1[i] = points3d1Mat.at<cv::Point3d>(i);
//	for (int i = 0; i < (int)points2.size(); i++) points3d2[i] = points3d2Mat.at<cv::Point3d>(i);
	for (int i = 0; i < (int)points2.size(); i++) err[i] = errMat.at<double>(i);
	return ret;
}

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
	cv::Mat & errMat
)
{
	int ret = 0;
	cv::Mat r4 = r4Mat2 * r4Mat1.inv(); 
	cv::Mat r3 = r4(cv::Rect(0, 0, 3, 3));
	cv::Mat tvec = r4(cv::Rect(3, 0, 1, 3));
	cv::Mat points3d1;
	cv::Mat points3d2;

	ret = triangulatePoints2(camMat1, disVec1, camMat2, disVec2, r3, tvec,
		points1, points2,
		points3d1, points3d2, errMat);

//	cout << "Triangulation error: \n" << errMat << endl;

	cv::Mat vecx1_4xN = cv::Mat::ones(4, points3d1.cols, CV_64F); 
	for (int i = 0; i < points3d1.cols; i++) {
		vecx1_4xN.at<double>(0, i) = points3d1.at<cv::Point3d>(i).x;
		vecx1_4xN.at<double>(1, i) = points3d1.at<cv::Point3d>(i).y;
		vecx1_4xN.at<double>(2, i) = points3d1.at<cv::Point3d>(i).z;
	}
	cv::Mat vecx1_global = r4Mat1.inv() * vecx1_4xN;
//	cout << "Vecx1:\n" << vecx1_global << endl;

//	cv::Mat vecx2_4xN = cv::Mat::ones(4, points3d2.cols, CV_64F);
//	for (int i = 0; i < points3d2.cols; i++) {
//		vecx2_4xN.at<double>(0, i) = points3d2.at<cv::Point3d>(i).x;
//		vecx2_4xN.at<double>(1, i) = points3d2.at<cv::Point3d>(i).y;
//		vecx2_4xN.at<double>(2, i) = points3d2.at<cv::Point3d>(i).z;
//	}
//	cv::Mat vecx2_global = r4Mat2.inv() * vecx2_4xN;
//	cout << "Vecx2:\n" << vecx2_global << endl;

	points3d = vecx1_global(cv::Rect(0, 0, vecx1_global.cols, 3)).t(); 

//	cout << "Points3d: \n" << points3d << endl;

	return ret; 
}

int triangulatePointsGlobalCoordinate(
	const cv::Mat & camMat1, 
	const cv::Mat & disVec1, 
	const cv::Mat & rmat1, 
	const cv::Mat & tvec1, 
	const cv::Mat & camMat2, 
	const cv::Mat & disVec2, 
	const cv::Mat & rmat2, 
	const cv::Mat & tvec2, 
	const cv::Mat & points1, 
	const cv::Mat & points2, 
	cv::Mat & points3d, cv::Mat & err)
{
	cv::Mat r41 = cv::Mat::eye(4, 4, rmat1.type()); 
	rmat1.copyTo(r41(cv::Rect(0, 0, 3, 3))); 
	tvec1.copyTo(r41(cv::Rect(3, 0, 1, 3))); 
	cv::Mat r42 = cv::Mat::eye(4, 4, rmat2.type());
	rmat2.copyTo(r42(cv::Rect(0, 0, 3, 3)));
	tvec2.copyTo(r42(cv::Rect(3, 0, 1, 3)));
	return
		triangulatePointsGlobalCoordinate(
			camMat1,
			disVec1,
			r41,
			camMat2,
			disVec2,
			r42,
			points1,
			points2,
			points3d, 
			err);
}
