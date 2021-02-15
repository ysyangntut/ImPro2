#define _CRT_SECURE_NO_WARNINGS     // Make Microsoft Visual Studio quiet on localtime() and many other functions

#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

#include "impro_util.h"
#include "FileSeq.h"

using namespace std;

cv::Point3f evalPolySurf(
	const cv::Mat& polyCoefs, // poly surface coefficients cv::Mat(numPoints, maxPolyOrder + 1, CV_64FC3)
	const double trial_x[]    // trial parameger, vector<dobule>(2)
);


/*!
	\brief This function returns the difference (in pixels) between an actual image point
	and a projected image point. 
	
	The projected image point is calculated by projecting a 3D point to an image coord. 
	The projection is based on a polynomial surface. 
	This function is the cost function for finding the 3D coordinate of an image point which
	lies on a polynomial surface. This function is designed for Polynomial Friction Pendulum 
	Isolators (PFPIs) image analysis. 

*/
//double costFuncPoly(
//	const cv::Mat& cmat,
//	const cv::Mat& dvec,
//	const cv::Mat& rvec,
//	const cv::Mat& tvec,
//	const cv::Mat polyCoefs, // (1, (maxPolyOrder + 1) *3, CV_64FC3) for polynomial 
//	const cv::Point2f& imgPoint, // a single image point 
//	const cv::Point3f& objRefPoint, // reference point, or origin of the polynomial coordinate
//	cv::Point3f& objPoint, // calculated object point (which has minimum error)
//	cv::Point2f& prjPoint  // projected point of the object point
//);

int FuncConstraintOnPolySurface(int argc, char** argv)
{
	std::string fnameTracking, fnameCamIntExt, fnameResult; 
	cv::FileStorage fsTracking, fsCamIntExt; 
	std::vector<std::vector<cv::Point2f> > vecvecTrackingData; // Dim: numSteps * numPoints
//	std::vector<cv::Point2f> vecImgShift; // image shift (in pixels) of each point. Dim: numPoints
	int numSteps, numPoints; 
	cv::Mat cmat, dvec, rvec, tvec; 
	std::vector<cv::Point3f> objPointsInit;  // initial object points of POI (entered by user)
	std::vector<cv::Point2f> prjPointsInit;  // initial image points of POI (projected)
	// Points of interests (POI) sometimes cannot be tracked because of bad image quality or being covered. 
	// POIs are represented by tracking points, assuming they have the same movement in videos. 
	std::vector<std::vector<cv::Point2f> > imgPointsHistory; // Dim: numSteps * numPoints. Image point history of points of interests, assuming rigid body motion
	std::vector<std::vector<cv::Point3f> > objPointsHistory;
	std::vector<std::vector<cv::Point2f> > prjPointsHistory;
	std::vector<std::vector<cv::Vec2d> >   trialHistory;
	cv::Mat polyCoefs; // numPoints * (maxPolyOrder + 2) * 3 (for each point, a8 ~ a0, b8 ~ b0, c8 ~ c0, ymax, ymin)
	std::vector<int> refPoints;
	std::vector<int> analysisOrdering;
	const int maxPolyOrder = 8; 

	// Get image sequence
	std::cout << "# Enter image sequence: \n";
	FileSeq fsq; 
	fsq.setDirFilesByConsole(); 

	//	Camera parameters
	while (true)
	{
		printf("# Enter camera parameter file (xml which has cameraMatrix, distortionVector, rvec, tvec): ");
		fnameCamIntExt = readStringLineFromCin();
		fsCamIntExt.open(fnameCamIntExt, cv::FileStorage::READ);
		if (fsCamIntExt.isOpened() == true)
			break;
		printf("# Cannot open the file %s.\n", fnameCamIntExt.c_str());
		printf("# Try again.\n");
	}
	fsCamIntExt["cameraMatrix"] >> cmat;
	std::cout << "# Camera matrix:\n" << cmat << std::endl;
	fsCamIntExt["distortionVector"] >> dvec;
	std::cout << "# Distortion vector:\n" << dvec << std::endl;
	fsCamIntExt["rvec"] >> rvec;
	std::cout << "# Rvec: \n" << rvec << std::endl;
	fsCamIntExt["tvec"] >> tvec;
	std::cout << "# Tvec: \n" << tvec << std::endl;


	// Load tracked data (from tmatch or other tracking method)
	// file format: <opencv_storage>
    //  	<?xml version = "1.0"?>
    //  		<opencv_storage>
    //  		<numSteps>1349</numSteps>
    //  		<numPoints>4</numPoints>
    //  		<VecVecPoint2f>
    //  		<_>

	while (true)
	{
		printf("# Enter the compact summary result of tracking: ");
		fnameTracking = readStringLineFromCin();
		fsTracking.open(fnameTracking, cv::FileStorage::READ);
		if (fsTracking.isOpened() == true) 
			break;
		printf("# Cannot open the file %s.\n", fnameTracking.c_str());
		printf("# Try again.\n"); 
	}
	fsTracking["VecVecPoint2f"] >> vecvecTrackingData;
	numSteps = (int) vecvecTrackingData.size();
	numPoints = (int) vecvecTrackingData[0].size();
	printf("# number of steps: %d \n", numSteps);
	printf("# number of points: %d \n", numPoints);
	for (int iStep = 1; iStep < numSteps; iStep++)
	{
		if (numPoints != vecvecTrackingData[iStep].size())
		{
			printf("# Warning: Time step %d (1-based) has %d points, which is not consistent.\n",
				(int) iStep + 1, (int) (vecvecTrackingData[iStep].size()));
		}
	}

	// # Enter world coordinate of points (which will be treated as origin of local coord. sys)
	std::cout << "# Enter world coordinate of points (which will be treated as origin of local coord. sys):\n";
	objPointsInit.resize(numPoints); 
	for (int iPoint = 0; iPoint < numPoints; iPoint++)
	{
		std::cout << "#   World coordinate of point " << iPoint + 1 << " (1-based):\n";
		objPointsInit[iPoint].x = (float) readDoubleFromIstream(std::cin); 
		objPointsInit[iPoint].y = (float) readDoubleFromIstream(std::cin);
		objPointsInit[iPoint].z = (float) readDoubleFromIstream(std::cin);
	}
	std::cout << "# Initial object points:\n" << objPointsInit << std::endl;
	
	// # Polygon contraints of each points 
	// See the function evalPolySurf() for the definition of the coefficients. 
	std::cout << "# Polygon contraints of each points (z = f(x, y))\n";
	std::cout << "#  z = c0												   \n";
	std::cout << "#    + a1 * (x - a0) ^ 1 + b1 * (y - b0) ^ 1 + c1 * r ^ 1 \n";
	std::cout << "#    + a2 * (x - a0) ^ 2 + b2 * (y - b0) ^ 2 + c2 * r ^ 2 \n";
	std::cout << "#    + a3 * (x - a0) ^ 3 + b3 * (y - b0) ^ 3 + c3 * r ^ 3 \n";
	std::cout << "#    ... 												   \n";
	std::cout << "#    + a8 * (x - a0) ^ 8 + b8 * (y - b0) ^ 8 + c8 * r ^ 8,\n";
	std::cout << "# where r = sqrt((x - x0) ^ 2 + (y - y0) ^ 2),\n"; 
	std::cout << "# and please enter: a8 b8 c8  a7 b7 c7 ... a0 b0 c0 z_min z_max\n";

	polyCoefs = cv::Mat::zeros(numPoints, maxPolyOrder + 2, CV_64FC3);
	for (int iPoint = 0; iPoint < numPoints; iPoint++)
	{
		for (int iPoly = maxPolyOrder; iPoly >= 0; iPoly--)
		{
			polyCoefs.at<cv::Vec3d>(iPoint, iPoly)[0] = readDoubleFromIstream(std::cin); // ai
			polyCoefs.at<cv::Vec3d>(iPoint, iPoly)[1] = readDoubleFromIstream(std::cin); // bi
			polyCoefs.at<cv::Vec3d>(iPoint, iPoly)[2] = readDoubleFromIstream(std::cin); // ci 
		}
		polyCoefs.at<cv::Vec3d>(iPoint, maxPolyOrder + 1)[0] = readDoubleFromIstream(std::cin); // zmin
		polyCoefs.at<cv::Vec3d>(iPoint, maxPolyOrder + 1)[1] = readDoubleFromIstream(std::cin); // zmax
	}
	std::cout << "# Polygon coefficients: \n" << polyCoefs << std::endl;

	// reference point(1 - based) (negative value means there is no reference point, or the reference point is fixed world coordinate)
	std::cout << "# Enter reference point: \n";
	refPoints.resize(numPoints); 
	for (int iPoint = 0; iPoint < numPoints; iPoint++)
	{
		printf("#  Reference point of point %d (1-based) (0 means it does not have reference point.:\n",
			iPoint + 1);
		refPoints[iPoint] = readIntFromCin(0, numPoints) - 1;
	}

	// Analysis ordering
	// Analysis ordering(Points with references need to be calculated after their reference points)
	std::cout << "# Analysis ordering.\n";
	std::cout << "# Points with references need to be calculated after their reference points.\n";
	analysisOrdering.resize(numPoints);
	for (int iPoint = 0; iPoint < numPoints; iPoint++)
	{
		analysisOrdering[iPoint] = readIntFromIstream(std::cin); 
	}

	// Output file
	std::cout << "# Output file of the polygon constraint analysis (CSV file):\n";
	fnameResult = readStringLineFromCin();

	// Get output visualization image sequence
	std::cout << "# Enter output visualization image sequence: \n";
	FileSeq fsqVis;
	fsqVis.setDirFilesByConsole();


	// Calculate initial image points of POI (projected) 
	cv::Mat imgInit; 
	imgInit = cv::imread(fsq.fullPathOfFile(0));
	for (int i = 0; i < numPoints; i++)
	{
		// 
		cv::projectPoints(
			objPointsInit,
			rvec, tvec, cmat, dvec,
			prjPointsInit);
		if (imgInit.rows > 0 && imgInit.cols > 0)
		{
			cv::Mat points(numPoints, 2, CV_32F); 
			for (int i = 0; i < numPoints; i++) {
				points.at<float>(i, 0) = (float)prjPointsInit[i].x;
				points.at<float>(i, 1) = (float)prjPointsInit[i].y;
			}
			drawPointsOnImage(imgInit, points, "+", 30, 10, cv::Scalar(0, 255, 255), 0.5); 
		}
	}
	if (imgInit.rows > 0 && imgInit.cols > 0)
	{
		imshow_resize("POI", imgInit, 0.3); 
		cv::waitKey(2000);
	}
	cv::destroyAllWindows(); 
	cout << "Projected POI: \n" << prjPointsInit << endl;

	// Calculate image points of POI (projected) (prjPointsHistory)
	imgPointsHistory = vecvecTrackingData;
	for (int i = 0; i < numPoints; i++)
	{
		for (int j = 0; j < numSteps; j++)
		{
			imgPointsHistory[j][i] = prjPointsInit[i]
				+ vecvecTrackingData[j][i] - vecvecTrackingData[0][i];
		}
	}

//	cout << "Target image point: " << imgPointsHistory[0][0] << endl;

	// allocate trial parameter history 
	trialHistory = std::vector<std::vector<cv::Vec2d> >(numSteps, std::vector<cv::Vec2d>(numPoints, cv::Vec2d(0., 0.))); 

	// For each frame
	for (int iStep = 0; iStep < numSteps; iStep++)
	{
		// Load image

		// Get image point --> vecvecTrackingData[iStep][iPoint]
		for (int iPoint = 0; iPoint < numPoints; iPoint++)
		{
			// Forget why this loop was created
		} // end of iPoint loop

		// Get 3D point
		bool debugIteration = false; 
		if (iStep == 302) {
			debugIteration = true; 
		}
		for (int iPointOrder = 0; iPointOrder < numPoints; iPointOrder++)
		{
			int iPoint = analysisOrdering[iPointOrder] - 1; 

			// Trial iterations
			cv::Point2f target_P2f;  // the actual image point of the target  
			double trial_x[2];       // initial guess
			static double trial_x_prev[999][2] = {}; 
			double delta_x;          // delta_x for estimating jacobian matrix
			cv::Point3f trial_P3f;   // Point3f based on trial_x
			cv::Point2f trial_P2f;   // Projected Point2f based on trial_P3f
			cv::Mat jacob = cv::Mat::zeros(2, 2, CV_64F); // jacobean matrix 
			double eig0, eig1; 
			int nMaxItr = 10;        // max number of iterations
			double tol = 1e-6;       // tolerance (norm(y_this - y_previous), in unit of pixel) 
			double conditionJ = -1;  // condition number of jacobi matrix.

			// Set actual image point target_P2f
//			int iPoint = 0;
//			int iStep = 100;
			target_P2f = imgPointsHistory[iStep][iPoint];
			if (debugIteration) cout << "Target image point is: " << target_P2f << "\n";

			// Set trial paramters and delta (the same unit of trial_x) 
			if (iStep == 0) {
//				trial_x[0] = 0.0;       // initial guess
//				trial_x[1] = 0.0;       // initial guess
//				For iStep == 0, we assume the initial guess is trial_x = (x0, y0), 
//				making (trial_x[0] - x0) and (trial_x[1] - y0) being zeros.
				trial_x[0] = polyCoefs.at<cv::Vec3d>(iPoint, 0)[0];
				trial_x[1] = polyCoefs.at<cv::Vec3d>(iPoint, 0)[1];
			}
			else {
				// comment: if iStep > 0, the trial_x should be the trial_x of the same point in the previous iStep
				// The problem is, the trial_x is shared with all points, not for any specific point, so the trial_x
				// must have been replaced by other points. 
				// We can not assume trial_x is the X and Y in the global coordinate, so we need to modify the code. 
				trial_x[0] = trialHistory[iStep - 1][iPoint][0];       // initial guess
				trial_x[1] = trialHistory[iStep - 1][iPoint][1];       // initial guess
				//trial_x[0] = trial_x_prev[iPoint][0];
				//trial_x[1] = trial_x_prev[iPoint][1];
			}
			delta_x = 0.2;

			for (int itr = 0; itr < nMaxItr; itr++)
			{
				// Print info
				if (debugIteration) cout << "----------------\nIteration " << itr << "\n-----------\n";

				// from trial_x to trial_P3f 
				if (debugIteration) {
					cout << "Trial parameters (x) are: \n";
					for (int i = 0; i < size(trial_x); i++)
						cout << trial_x[i] << endl;
				}

				// estimate the point coordinate on the surface based on parameters of trial_x[0] and trial_x[1] 
				trial_P3f = evalPolySurf(polyCoefs(cv::Rect(0, iPoint, polyCoefs.cols, 1)), trial_x);

				// if the polynomial surface is relative to a reference point, 
				// convert it from relative coordinate to world coordinate
				int theRefPoint = refPoints[iPoint];
				if (theRefPoint < 0) {
					// if this poly surface is based on world coordinate (without a reference point)
				}
				else {
					trial_P3f += objPointsHistory[iStep][theRefPoint]; 
				}
				if (debugIteration) cout << "Trial point 3d: " << trial_P3f << "\n";

				// from trial_P3f to trial_P2f
				vector<cv::Point3f> tmpObjPoints(1);
				vector<cv::Point2f> tmpPrjPoints(1);
				tmpObjPoints[0] = trial_P3f;
				cv::projectPoints(
					tmpObjPoints,
					rvec, tvec, cmat, dvec,
					tmpPrjPoints);
				trial_P2f = tmpPrjPoints[0];
					if (debugIteration) cout << "Projectd point of trial point:" << trial_P2f << "\n";

				// find err (trial_y =  trial_P2f - target_P2f)
				double trial_y[2];
				trial_y[0] = (double)trial_P2f.x - (double)target_P2f.x;
				trial_y[1] = (double)trial_P2f.y - (double)target_P2f.y;
				if (debugIteration) {
					cout << "Equiv. trial y are: \n";
					for (int i = 0; i < size(trial_y); i++)
						cout << trial_y[i] << endl;
				}
				double ysq = trial_y[0] * trial_y[0] + trial_y[1] * trial_y[0];
				if (ysq < tol * tol)
				{
					if (debugIteration) cout << "Breaking Newton loop because of norm of y is " << sqrt(ysq) << "\n";
					break;
				}
				static double trial_y_prev[2];
				if (itr == 0) {
					trial_y_prev[0] = trial_y[0];
					trial_y_prev[0] = trial_y[1];
				}
				else {
					// calculate change of y. 
					// save the current trial_y for later trial_y_prev
					double dysq_0 = (trial_y_prev[0] - trial_y[0]);
					double dysq_1 = (trial_y_prev[1] - trial_y[1]);
					double dysq_sq = dysq_0 * dysq_0 + dysq_1 * dysq_1;
					if (dysq_sq < tol * tol)
					{
						if (debugIteration) cout << "Breaking Newton loop because of norm of change of y is " << sqrt(dysq_sq) << "\n";
						break;
					}
				}  // end of if itr == 0. 


				// Jacobi matrix
				if (true)
				{
					cv::Mat jacob_prev;
					if (itr > 0) jacob_prev = jacob.clone();
					jacob = cv::Mat::zeros(2, 2, CV_64F);
					double trial_x_vari[2];
					std::vector<cv::Point3f> trial_P3f_vari(2);   // Point3f based on trial_x
					std::vector<cv::Point2f> trial_P2f_vari(2);   // Projected Point2f based on trial_P3f

					// Jacobi matrix (0): Set trial paramters and delta (the same unit of trial_x) 
					trial_x_vari[0] = trial_x[0] + delta_x;
					trial_x_vari[1] = trial_x[1];

					// Jacobi matrix (0): from trial_x_vari to trial_P3f_vari
					if (debugIteration) {
						cout << "Trial parameters (for Jacobi) are: \n";
						for (int i = 0; i < size(trial_x); i++)
							cout << trial_x_vari[i] << endl;
					}
					trial_P3f_vari[0] = evalPolySurf(polyCoefs(cv::Rect(0, iPoint, polyCoefs.cols, 1)), trial_x_vari);
					// if the polynomial surface is relative to a reference point, 
					// convert it from relative coordinate to world coordinate
					if (theRefPoint < 0) {
						// if this poly surface is based on world coordinate (without a reference point)
					}
					else {
						trial_P3f_vari[0] += objPointsHistory[iStep][theRefPoint];
					}
					if (debugIteration) cout << "Trial point 3d (for Jacobi) (1): " << trial_P3f_vari[0] << "\n";

					// Jacobi matrix (1): Set trial paramters and delta (the same unit of trial_x) 
					trial_x_vari[0] = trial_x[0];
					trial_x_vari[1] = trial_x[1] + delta_x;

					// Jacobi matrix (1): from trial_x_vari to trial_P3f_vari
					if (debugIteration) {
						cout << "Trial parameters (for Jacobi) are: \n";
						for (int i = 0; i < size(trial_x); i++)
							cout << trial_x_vari[i] << endl;
					}
					trial_P3f_vari[1] = evalPolySurf(polyCoefs(cv::Rect(0, iPoint, polyCoefs.cols, 1)), trial_x_vari);
					if (theRefPoint < 0) {
						// if this poly surface is based on world coordinate (without a reference point)
					}
					else {
						trial_P3f_vari[1] += objPointsHistory[iStep][theRefPoint];
					}
					if (debugIteration) cout << "Trial point 3d (for Jacobi) (1): " << trial_P3f_vari[1] << "\n";

					// from trial_P3f_vari to trial_P2f_vari
					cv::projectPoints(
						trial_P3f_vari,
						rvec, tvec, cmat, dvec,
						trial_P2f_vari);
					if (debugIteration) cout << "Projectd point of trial point variations:\n" << trial_P2f_vari[0] << "\n" << trial_P2f_vari[1] << "\n";

					// Jacobi matrix: final
					jacob.at<double>(0, 0) = ((double)trial_P2f_vari[0].x - (double)trial_P2f.x) / delta_x;
					jacob.at<double>(1, 0) = ((double)trial_P2f_vari[0].y - (double)trial_P2f.y) / delta_x;
					jacob.at<double>(0, 1) = ((double)trial_P2f_vari[1].x - (double)trial_P2f.x) / delta_x;
					jacob.at<double>(1, 1) = ((double)trial_P2f_vari[1].y - (double)trial_P2f.y) / delta_x;
					if (debugIteration) cout << "Jacobi matrix: \n" << jacob << endl;

					// check eigenvalues of Jacobi. 
					cv::Mat eigval, eigvec;
					bool retEigen = cv::eigen(jacob, eigval, eigvec);
					eig0 = eigval.at<double>(0);
					eig1 = eigval.at<double>(1);
					if (debugIteration) {
						cout << "Eigenvalues:\n" << eigval << endl;
						cout << "Eigenvecs:\n" << eigvec << endl;
					}
					if (eig0 == 0.0 || eig1 == 0.0)
					{
						cerr << "Warning: Jacobian matrix is singular.\n";
					}
					else {
						if (eig0 > eig1)
							conditionJ = std::abs(eig0 / eig1);
						else
							conditionJ = std::abs(eig1 / eig0);
						if (debugIteration) cout << "Jacobi condition number is " << conditionJ << "\n";
					} // end of if jacobi is singular
				} // end of if jacobi needs to be updated

				// estimate next trial_x:  trial_x_next = trial_x - (J.t * J).inv * J.t * trial_y
				cv::Mat trial_dx(2, 1, CV_64F), trial_dy(2, 1, CV_64F);
				trial_dy.at<double>(0) = trial_y[0];
				trial_dy.at<double>(1) = trial_y[1];
				if (eig0 != 0.0 && eig1 != 0.0 && conditionJ < 1000.0)
				{
					// well-condition
					trial_dx = -((jacob.t() * jacob).inv() * jacob.t()) * trial_dy;
				}
				else {
					// ill-condition
					double lambda = 1; 
					trial_dx = -((jacob.t() * jacob + lambda * cv::Mat::eye(2, 2, CV_64F)).inv() * jacob.t()) * trial_dy;
				}
				if (debugIteration) cout << "Trial_dx:\n" << trial_dx << "\n";

				// Next trial_x 
				if (itr + 1 < nMaxItr) {
					trial_x[0] += trial_dx.at<double>(0);
					trial_x[1] += trial_dx.at<double>(1);
				}
			} // end of Newton's iteration (itr < nMaxItr) 

			// check memory allocation
			if (objPointsHistory.size() < numSteps)
			{
				if (debugIteration)
					cout << "# Allocating objPointsHistory.\n";
				objPointsHistory.resize(numSteps,
					std::vector<cv::Point3f>(numPoints, cv::Point3f(0.f, 0.f, 0.f))); 
			} 
			if (objPointsHistory[iStep].size() < numPoints)
			{
				objPointsHistory[iStep].resize(numPoints);
			}
			if (prjPointsHistory.size() < numSteps)
			{
				if (debugIteration)
					cout << "# Allocating prjPointsHistory.\n";
				prjPointsHistory.resize(numSteps,
					std::vector<cv::Point2f>(numPoints, cv::Point2f(0.f, 0.f)));
			}
			if (prjPointsHistory[iStep].size() < numPoints)
			{
				prjPointsHistory[iStep].resize(numPoints);
			}

			// set calculated object point to objPointsHistory[iStep][iPoint]
			objPointsHistory[iStep][iPoint] = trial_P3f; 
			prjPointsHistory[iStep][iPoint] = trial_P2f;
			trialHistory[iStep][iPoint][0] = trial_x[0];
			trialHistory[iStep][iPoint][1] = trial_x[1];

			// 
			printf("(%7.1f %7.1f %7.1f) ", trial_P3f.x, trial_P3f.y, trial_P3f.z);
			if (iPointOrder == numPoints - 1)
				printf("\n");
		} // end of iPointOrder loop 

		// Visualization
		bool doVis = true; 
		//if (iStep == 0 || iStep == 590) {
		//	doVis = true;
		//}
		if (doVis)
		{
			//  Load image 
			cv::Mat imgVis = cv::imread(fsq.fullPathOfFile(iStep));

			// arrays of points
			vector<cv::Point3f> tmpObjPoints;
			vector<cv::Point2f> tmpPrjPoints;
			vector<cv::Point> tmpPoint;

			// draw contour on poly surfaces
			for (int iPoint = 0; iPoint < numPoints; iPoint++)
			{
				int theRefPoint = refPoints[iPoint];
				if (theRefPoint < 0) continue; // only plot point which has a reference point
				float rad[] = { 20, 40, 60, 80, 100, 120, 140, 160, 180, 200 };
				for (int ir = 0; ir < size(rad); ir++)
				{
					tmpObjPoints.resize(2);
					tmpPrjPoints.resize(2);
					tmpPoint.resize(2);
					for (int ith = 0; ith < 60; ith++)
					{
						double theta1 = ith * 2. * 3.1416 / 60.;
						double theta2 = (ith + (int) 1) * 2. * 3.1416 / 60.;
						double xy1[2], xy2[2];
						xy1[0] = rad[ir] * cos(theta1) + polyCoefs.at<cv::Vec3d>(iPoint, 0)[0]; 
						xy1[1] = rad[ir] * sin(theta1) + polyCoefs.at<cv::Vec3d>(iPoint, 0)[1];
						xy2[0] = rad[ir] * cos(theta2) + polyCoefs.at<cv::Vec3d>(iPoint, 0)[0];
						xy2[1] = rad[ir] * sin(theta2) + polyCoefs.at<cv::Vec3d>(iPoint, 0)[1];
						tmpObjPoints[0] = evalPolySurf(polyCoefs(cv::Rect(0, iPoint, polyCoefs.cols, 1)), xy1);
						tmpObjPoints[1] = evalPolySurf(polyCoefs(cv::Rect(0, iPoint, polyCoefs.cols, 1)), xy2);
						tmpObjPoints[0] += objPointsHistory[iStep][theRefPoint];
						tmpObjPoints[1] += objPointsHistory[iStep][theRefPoint];
						cv::projectPoints(
							tmpObjPoints,
							rvec, tvec, cmat, dvec,
							tmpPrjPoints);
						tmpPoint[0] = cv::Point((int) (tmpPrjPoints[0].x + .5f), (int)(tmpPrjPoints[0].y + .5f));
						tmpPoint[1] = cv::Point((int) (tmpPrjPoints[1].x + .5f), (int)(tmpPrjPoints[1].y + .5f));
						cv::line(imgVis, tmpPoint[0], tmpPoint[1], cv::Scalar(128, 0, 0), 2);
					}
				} // end of ir (index of radius)
			} // end of iPointOrder
			  
			// 
			//  Plot image tracking points (where the image feature is) as green points
			for (int iPoint = 0; iPoint < numPoints; iPoint++)
			{
				int boxSize = 6; 
				int thick = 6; 
				cv::Point p1((int)(vecvecTrackingData[iStep][iPoint].x + .5f), (int)(vecvecTrackingData[iStep][iPoint].y + .5f));
				cv::Point p2 = p1; 
				p1 -= cv::Point(boxSize, boxSize);
				p2 += cv::Point(boxSize, boxSize);
				cv::rectangle(imgVis, p1, p2, cv::Scalar(0, 255, 0), thick);
			}
			//
			//  Plot image points ( Image point history of points of interests, assuming rigid body motion with tracked points)
			//  as blue points
			for (int iPoint = 0; iPoint < numPoints; iPoint++)
			{
				int boxSize = 6;
				int thick = 6; 
				cv::Point p1((int)(imgPointsHistory[iStep][iPoint].x + .5f), (int)(imgPointsHistory[iStep][iPoint].y + .5f));
				cv::Point p2 = p1;
				p1 -= cv::Point(boxSize, boxSize);
				p2 += cv::Point(boxSize, boxSize);
				cv::rectangle(imgVis, p1, p2, cv::Scalar(255, 0, 0), thick);
			}
			//  Plot lines between track/img points
			for (int iPoint = 0; iPoint < numPoints; iPoint++)
			{
				int thick = 6;
				cv::Point p1((int)(vecvecTrackingData[iStep][iPoint].x + .5f), (int)(vecvecTrackingData[iStep][iPoint].y + .5f));
				cv::Point p2((int)(imgPointsHistory[iStep][iPoint].x + .5f),   (int)(imgPointsHistory[iStep][iPoint].y + .5f));
				cv::line(imgVis, p1, p2, cv::Scalar(128, 128, 0), thick);
			}
			//  Plot projected points
			//  Plot image points ( Image point history of points of interests, assuming rigid body motion with tracked points)
			//  as red points
			for (int iPoint = 0; iPoint < numPoints; iPoint++)
			{
				int boxSize = 4;
				int thick = 4;
				cv::Point p1((int)(prjPointsHistory[iStep][iPoint].x + .5f), (int)(prjPointsHistory[iStep][iPoint].y + .5f));
				cv::Point p2 = p1;
				p1 -= cv::Point(boxSize, boxSize);
				p2 += cv::Point(boxSize, boxSize);
				cv::rectangle(imgVis, p1, p2, cv::Scalar(0, 0, 255), thick);
			}

			// draw axes on table
			int thick = 2; 
			tmpObjPoints.resize(8);
			tmpPrjPoints.resize(8);
			tmpPoint.resize(8);
			tmpObjPoints[0] = cv::Point3f(0, 0, 0);
			tmpObjPoints[1] = cv::Point3f(100.f, 0, 0);
			tmpObjPoints[2] = cv::Point3f(0, 100.f, 0);
			tmpObjPoints[3] = cv::Point3f(0, 0, 100.f);
			tmpObjPoints[4] = cv::Point3f(0, 3500.f, 0);
			tmpObjPoints[5] = cv::Point3f(100.f, 3500.f, 0);
			tmpObjPoints[6] = cv::Point3f(0, 3600.f, 0);
			tmpObjPoints[7] = cv::Point3f(0, 3500.f, 100.f); 
			cv::projectPoints(
				tmpObjPoints,
				rvec, tvec, cmat, dvec,
				tmpPrjPoints);
			for (int iPoint = 0; iPoint < 8; iPoint++) 
				tmpPoint[iPoint] = cv::Point((int)(tmpPrjPoints[iPoint].x + .5f), (int)(tmpPrjPoints[iPoint].y + .5f));
			cv::line(imgVis, tmpPoint[0], tmpPoint[1], cv::Scalar(0, 0, 128), thick);
			cv::line(imgVis, tmpPoint[0], tmpPoint[2], cv::Scalar(0, 128, 0), thick);
			cv::line(imgVis, tmpPoint[0], tmpPoint[3], cv::Scalar(128, 0, 0), thick);
			cv::line(imgVis, tmpPoint[4], tmpPoint[5], cv::Scalar(0, 0, 128), thick);
			cv::line(imgVis, tmpPoint[4], tmpPoint[6], cv::Scalar(0, 128, 0), thick);
			cv::line(imgVis, tmpPoint[4], tmpPoint[7], cv::Scalar(128, 0, 0), thick);

			// show image
			imshow_resize("Visualization", imgVis, 0.3);
			int ikey = cv::waitKey(1);
			cv::imwrite(fsqVis.fullPathOfFile(iStep), imgVis);
			if (ikey == 27) {
				cv::destroyAllWindows();
				break;
			}
		} // end of if doVis 
	}

	// output to csv
	std::FILE * fResultCsv;
	fResultCsv = fopen(fnameResult.c_str(), "w");
	if (fResultCsv == NULL) {
		std::cerr << "Cannot write to file " << fnameResult << "\n";
		return 0; 
	}
	for (int iStep = 0; iStep < numSteps; iStep++)
	{
		for (int iPoint = 0; iPoint < numPoints; iPoint++)
		{
			fprintf(fResultCsv, "%8.2f, %8.2f, %8.2f, ,",
				objPointsHistory[iStep][iPoint].x,
				objPointsHistory[iStep][iPoint].y,
				objPointsHistory[iStep][iPoint].z);
		}
		fprintf(fResultCsv, "\n"); 
	}
	fclose(fResultCsv); 

	return 0; 
}

// This function returns a point Point3f(x, y, z) which is on the 
// polynomial surface defined by the coeffocients. 
// The x, y, z are calculated by:
// x = trial_x[0], 
// y = trial_x[1], and 
// z = z0
//   + a1 * (x - x0) ^ 1 + b1 * (y - y0) ^ 1 + c1 * r ^ 1
//   + a2 * (x - x0) ^ 2 + b2 * (y - y0) ^ 2 + c2 * r ^ 2
//   + a3 * (x - x0) ^ 3 + b3 * (y - y0) ^ 3 + c3 * r ^ 3
//   ... 
//   + a8 * (x - x0) ^ 8 + b8 * (y - y0) ^ 8 + c8 * r ^ 8, 
// where
// x0 = polyCoefs.at<cv::Vec3d>(0, 0)[0],
// y0 = polyCoefs.at<cv::Vec3d>(0, 0)[1],
// z0 = polyCoefs.at<cv::Vec3d>(0, 0)[2],
// r = sqrt((x - x0) ^ 2 + (y - y0) ^ 2), 
// a1 = polyCoefs.at<cv::Vec3d>(0, 1)[0],
// b1 = polyCoefs.at<cv::Vec3d>(0, 1)[1],
// c1 = polyCoefs.at<cv::Vec3d>(0, 1)[2],
// a2 = polyCoefs.at<cv::Vec3d>(0, 2)[0],
// b2 = polyCoefs.at<cv::Vec3d>(0, 2)[1],
// c2 = polyCoefs.at<cv::Vec3d>(0, 2)[2],
// ...
// a8 = polyCoefs.at<cv::Vec3d>(0, 8)[0],
// b8 = polyCoefs.at<cv::Vec3d>(0, 8)[1],
// c8 = polyCoefs.at<cv::Vec3d>(0, 8)[2],
cv::Point3f evalPolySurf(
	const cv::Mat& polyCoefs, // poly surface coefficients cv::Mat(1, maxPolyOrder + 2, CV_64FC3)   (the last column is [0]:zmin, [1]:zmax)
	const double trial_x[]   // trial parameger, vector<dobule>(2)
)
{
	//double x = trial_x[0];
	//double y = trial_x[1];
	//double r = sqrt(trial_x[0] * trial_x[0] + trial_x[1] * trial_x[1]);
	double x = trial_x[0], y = trial_x[1];
	double x_ = (x - polyCoefs.at<cv::Vec3d>(0, 0)[0]);
	double y_ = (y - polyCoefs.at<cv::Vec3d>(0, 0)[1]);
	double r = sqrt(x_ * x_ + y_ * y_); 
	double z = 0.0;
	int iPoly = 0;
	//z = polyCoefs.at<cv::Vec3d>(0, iPoly)[0] + polyCoefs.at<cv::Vec3d>(0, iPoly)[1] + polyCoefs.at<cv::Vec3d>(0, iPoly)[2];
	z += polyCoefs.at<cv::Vec3d>(0, iPoly)[2];
	double xpow = 1., ypow = 1., rpow = 1.;
	int iPoint = 0;
	for (int iPoly = 1; iPoly < polyCoefs.cols - 1; iPoly++)
	{
		xpow *= x_;
		ypow *= y_;
		rpow *= r;
		z += polyCoefs.at<cv::Vec3d>(iPoint, iPoly)[0] * xpow
			+ polyCoefs.at<cv::Vec3d>(iPoint, iPoly)[1] * ypow
			+ polyCoefs.at<cv::Vec3d>(iPoint, iPoly)[2] * rpow;
	}
	double zmin = polyCoefs.at<cv::Vec3d>(iPoint, polyCoefs.cols - 1)[0]; 
	double zmax = polyCoefs.at<cv::Vec3d>(iPoint, polyCoefs.cols - 1)[1];
	if (z < zmin) z = zmin;
	if (z > zmax) z = zmax; 

	return cv::Point3f((float)x, (float)y, (float)z);
}
