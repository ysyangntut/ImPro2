#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>

#include <opencv2/opencv.hpp>

using namespace std; 

//! 
/*! 
	\brief Calculates story displacement ux, uy, and tortion according to image points of upper and lower floors
	\param input			objPoints 3D points (N, 1, CV_64FC3) before story displacement ux, uy, tortion 
	\param center			the center point of tortion (note: move by ux uy first, rotate later. Center point moves with ux and uy)
	\param disp				displacement (3,1,CV_64F), (0,0):ux, (1,0):uy, (2,0):tortion in degrees
	\param newObjPoints		new (moved) 3D points after story displacement
	\return zero
*/
int calcStoryDispNewObjPoints(
	cv::Mat objPoints,  // 3D points (N, 1, CV_64FC3) before story displacement ux, uy, tortion 
	cv::Point3d center, // center point of tortion
	cv::Mat disp,       // displacement (3,1,CV_64F), (0,0):ux, (1,0):uy, (2,0):tortion in degrees
	cv::Mat& newObjPoints // new (moved) 3D points after story displacement 
); 

//! 
/*!
	\brief Function updatedRvecTvecByCameraRotation() updates rvec and tvec induced by camera rotation
	\param rvec original rvec before camera rotation cv::Mat(3, 1, CV_64F) 
	\param tvec original tvec before camera rotation cv::Mat(3, 1, CV_64F)
	\param camr original camr camera rotation vector cv::Mat(3, 1, CV_64f) (in radians)
	\param newRvec updated rvec before camera rotation cv::Mat(3, 1, CV_64F)
	\param newTvec updated tvec before camera rotation cv::Mat(3, 1, CV_64F)

*/
void updatedRvecTvecByCameraRotation(
	cv::Mat rvec,  // original rvec (3, 1, CV_64F) 
	cv::Mat tvec,  // original tvec (3, 1, CV_64F)
	cv::Mat camr,  // camera rotation (3, 1, CV_64F)
	cv::Mat& newRvec,  // updated rvec (3, 1, CV_64F)
	cv::Mat& newTvec)  // updated tvec (3, 1, CV_64F)
	;

int test_camRotationMatrixAndVector(); 

/*!
	\brief cost function of story displacement (drifts ux and uy, and tortion) 
	\param cmat					camera matrix of intrinsic parameters, cv::Mat(3, 3, CV_32F)
	\param dvec					camera distortion vector of intrinsic parameters, cv::Mat(1, 4 or 5 or 8, CV_32F)
	\param rvec					rotational vector of camera extrinsic parameters, cv::Mat(3, 1, CV_64F)
	\param tvec					translational vector of camera extrinsic parameters, cv::Mat(3, 1, CV_64F)
	\param refObjPointsMat		object points of reference points (supposed to be fixed on lower story), cv::Mat(N1, 1, CV_64FC3)
	\param trkObjPointsMat		object points of tracking points (moving points on upper story), cv::Mat(N1, 1, CV_64FC3)
	\param newRefImgPointsMat	new reference image points (on lower story) , cv::Mat(N1, 1, CV_32FC2)
	\param newTrkImgPointsMat   new traking image points (on upper story), cv::Mat(N2, 1, CV_32FC2)
	\param tortionCenter		tortional center, typically, the center of upper floor (user needs to decide it) 
	\param x					Input: newDisp and camRot. cv::Mat(6, 1, CV_64F)
	\param y					Output: resicual vector.   cv::Mat(2 * # of all points, 1, CV_64F)
*/
vector<double> costFuncStoryDispV4(
	cv::Mat cmat, // camera matrix of intrinsic parameters, cv::Mat(3, 3, CV_32F)
	cv::Mat dvec, // camera distortion vector of intrinsic parameters, cv::Mat(1, 4 or 5 or 8, CV_32F)
	cv::Mat rvec, // rotational vector of camera extrinsic parameters, cv::Mat(3, 1, CV_64F)
	cv::Mat tvec, // translational vector of camera extrinsic parameters, cv::Mat(3, 1, CV_64F)
	cv::Mat refObjPointsMat,  // object points of reference points(supposed to be fixed on with camera), cv::Mat(N1, 1, CV_64FC3)
	cv::Mat trkObjPointsMat,  // object points of tracking points(not fixed with camera), cv::Mat(N2, 1, CV_64FC3)
	cv::Mat newRefImgPointsMat,  // new reference image points(supposed to be fixed on with camera), cv::Mat(N1, 1, CV_32FC2)
	cv::Mat newTrkImgPointsMat,  // new traking image points(not fixed with camera), cv::Mat(N2, 1, CV_32FC2)
	cv::Point3d tortionCenter,  // tortional center, typically, the center of upper floor(user needs to decide it)
	cv::Mat & newProjRefImgPointsMat, // new projected reference image points (fixed with camera), cv::Mat(N1, 1, CV_32FC2)
	cv::Mat & newProjTrkImgPointsMat, // new projected tracking image points (not fixed with camera), cv::Mat(N2, 1, CV_32FC2)
	cv::Mat & newTrkObjPointsMat, // new traking object points (not fixed with camera), cv::Mat(N2, 1, CV_64FC3)
	cv::Mat x,	// Trial input: newDisp and camRot.cv::Mat(6, 1, CV_64F)
	cv::Mat & y // Output : resicual vector.cv::Mat(2 * # of all points (i.e., 2 * (N1 + N2), 1, CV_64F)
	)
{
	cv::Mat newDisp(3, 1, CV_64F); 
	cv::Mat camRot(3, 1, CV_64F);
	cv::Mat newTvec, newRvec, r4, r3, camRotR3(3, 3, CV_64F); 
	cv::Mat refProjError, trkProjError; 

	newTrkObjPointsMat = cv::Mat(trkObjPointsMat.rows, trkObjPointsMat.cols, trkObjPointsMat.type());
	newProjRefImgPointsMat = cv::Mat(newRefImgPointsMat.rows, newRefImgPointsMat.cols, newRefImgPointsMat.type());
	newProjTrkImgPointsMat = cv::Mat(newTrkImgPointsMat.rows, newTrkImgPointsMat.cols, newTrkImgPointsMat.type());

	newDisp.at<double>(0, 0) = x.at<double>(0); 
	newDisp.at<double>(1, 0) = x.at<double>(1);
	newDisp.at<double>(2, 0) = x.at<double>(2);
	camRot.at<double>(0, 0) = x.at<double>(3);
	camRot.at<double>(1, 0) = x.at<double>(4);
	camRot.at<double>(2, 0) = x.at<double>(5);

	// calculate newRvec and newTvec due to camRot
	updatedRvecTvecByCameraRotation(rvec, tvec, camRot, newRvec, newTvec); 

	// projection 
	calcStoryDispNewObjPoints(trkObjPointsMat, tortionCenter, newDisp, newTrkObjPointsMat);
	cv::projectPoints(refObjPointsMat, newRvec, newTvec, cmat, dvec, newProjRefImgPointsMat);
	cv::projectPoints(newTrkObjPointsMat, newRvec, newTvec, cmat, dvec, newProjTrkImgPointsMat);
	newProjRefImgPointsMat.convertTo(newProjRefImgPointsMat, newRefImgPointsMat.type());
	newProjTrkImgPointsMat.convertTo(newProjTrkImgPointsMat, newTrkImgPointsMat.type());
	if (newRefImgPointsMat.cols > newRefImgPointsMat.rows) newRefImgPointsMat = newRefImgPointsMat.t();
	if (newTrkImgPointsMat.cols > newTrkImgPointsMat.rows) newTrkImgPointsMat = newTrkImgPointsMat.t();
	if (newProjRefImgPointsMat.cols > newProjRefImgPointsMat.rows) newProjRefImgPointsMat = newProjRefImgPointsMat.t();
	if (newProjTrkImgPointsMat.cols > newProjTrkImgPointsMat.rows) newProjTrkImgPointsMat = newProjTrkImgPointsMat.t();
	// find residual 
	refProjError = newProjRefImgPointsMat - newRefImgPointsMat;
	trkProjError = newProjTrkImgPointsMat - newTrkImgPointsMat;
	// residual
	y = cv::Mat::zeros(refProjError.rows * 2 + trkProjError.rows * 2, 1, CV_64F);
	int yi = 0; 
	for (int i = 0; i < refProjError.rows; i++) {
		y.at<double>(yi, 0) = refProjError.at<cv::Point2f>(i, 0).x;
		yi += 1;
		y.at<double>(yi, 0) = refProjError.at<cv::Point2f>(i, 0).y;
		yi += 1;
	}
	for (int i = 0; i < trkProjError.rows; i++) {
		y.at<double>(yi, 0) = trkProjError.at<cv::Point2f>(i, 0).x;
		yi += 1;
		y.at<double>(yi, 0) = trkProjError.at<cv::Point2f>(i, 0).y;
		yi += 1;
	}
	// copy y (Mat) to residual (vector<double>) 
	vector<double> residual(y.rows);
	for (int i = 0; i < y.rows; i++)
		residual[i] = y.at<double>(i, 0);
	return residual;
}
/*!
	\brief Function estimateStoryDispWithoutReturningZeroV4() estimates story displacement function of story displacement 
	\param cmat					camera matrix of intrinsic parameters, cv::Mat(3, 3, CV_32F)
	\param dvec					camera distortion vector of intrinsic parameters, cv::Mat(1, 4 or 5 or 8, CV_32F)
	\param rvec					rotational vector of camera extrinsic parameters, cv::Mat(3, 1, CV_64F)
	\param tvec					translational vector of camera extrinsic parameters, cv::Mat(3, 1, CV_64F)
	\param refObjPoints         reference points in world coord. Can be vector<cv::Point3d> or cv::Mat(N1,1,CV_64FC3)
	\param trkObjPoints         tracking points in world coord. Can be vector<cv::Point3d> or cv::Mat(N2,1,CV_64FC3)
	\param newRefImgPoints      moved reference points in image coord. vector<cv::Point2f> or cv::Mat(N1,1,CV_32FC2)
	\param newTrkImgPoints      moved tracking points in image coord. vector<cv::Point2f> or cv::Mat(N2,1,CV_32FC2)
	\param tortionCenter        tortional center point in world coord. cv::Point3d. 
	\param camRot               camera rotational movement (3, 1, CV_64F)
	\param newDisp              displacement of story (ux, uy, tortion) (3, 1, CV_64F). (tortion unit is degrees)  
	\param newTrkObjPoints      moved tracking points in world coord. cv::Mat(N2,1,CV_64FC3)
	\param newProjRefImgPoints  new projection reference points in image coord (N1, 1, CV_32FC2)
	\param newProjTrkImgPoints  new projection tracking points in image coord (N2, 1, CV_32FC2)
*/
// Note:In this function estimateStoryDispWithoutReturningZeroV4(), 
// You may expect if you input all initial points, you will get zero cam movement and zero story displacement, 
// but actually not because it could use different way to do regression with the way OpenCV get rvec and tvec.
// Call estimateStoryDisp() if you want to do zero returning. 
int estimateStoryDispWithoutReturningZeroV4(
	cv::Mat cmat, // camera matrix of intrinsic parameters, cv::Mat(3, 3, CV_32F)
	cv::Mat dvec, // camera distortion vector of intrinsic parameters, cv::Mat(1, 4 or 5 or 8, CV_32F)
	cv::Mat rvec, // rotational vector of camera extrinsic parameters, cv::Mat(3, 1, CV_64F)
	cv::Mat tvec, // translational vector of camera extrinsic parameters, cv::Mat(3, 1, CV_64F)
	//cv::InputArray refImgPoints,     // reference points in image coord. Can be vector<cv::Point2f> or cv::Mat(N,1,CV_32FC2)
	//cv::InputArray trkImgPoints,     // tracking points in image coord. Can be vector<cv::Point2f> or cv::Mat(N,1,CV_32FC2)
	cv::InputArray refObjPoints,     // reference points in world coord. Can be vector<cv::Point3d> or cv::Mat(N,1,CV_64FC3)
	cv::InputArray trkObjPoints,     // tracking points in world coord. Can be vector<cv::Point3d> or cv::Mat(N,1,CV_64FC3)
	cv::InputArray newRefImgPoints,  // moved reference points in image coord. vector<cv::Point2f> or cv::Mat(N,1,CV_32FC2)
	cv::InputArray newTrkImgPoints,  // moved tracking points in image coord. vector<cv::Point2f> or cv::Mat(N,1,CV_32FC2)
	cv::Point3d    tortionCenter,    // tortional center point in world coord. 
	cv::Mat & camRot,                // camera rotational movement (3, 1, CV_64F)
	cv::Mat & newDisp,               // displacement of story (ux, uy, tortion) (3, 1, CV_64F) 
	cv::Mat & newTrkObjPoints,       // moved tracking points in world coord. cv::Mat(N,1,CV_64FC3)
	cv::Mat & newProjRefImgPoints,   // new projection reference points in image coord.  (N1, 1, CV_32FC2)
	cv::Mat & newProjTrkImgPoints    // new projection tracking points in image coord.  (N2, 1, CV_32FC2)
)
{
	bool debug = false; 
	//cv::Mat refImgPointsMat    = refImgPoints.getMat();
	//cv::Mat trkImgPointsMat    = trkImgPoints.getMat(); 
	cv::Mat refObjPointsMat    = refObjPoints.getMat();
	cv::Mat trkObjPointsMat    = trkObjPoints.getMat();
	cv::Mat newRefImgPointsMat = newRefImgPoints.getMat();
	cv::Mat newTrkImgPointsMat = newTrkImgPoints.getMat(); 
	cv::Mat newProjRefImgPointsMat = cv::Mat::zeros(refObjPointsMat.rows * refObjPointsMat.cols, 1, CV_32FC2);
	cv::Mat newProjTrkImgPointsMat = cv::Mat::zeros(trkObjPointsMat.rows * trkObjPointsMat.cols, 1, CV_32FC2);
	newTrkObjPoints = cv::Mat::zeros(refObjPointsMat.rows * refObjPointsMat.cols, 1, CV_64FC3);

	// check arguments types and sizes
	if (cmat.rows != 3 || cmat.cols != 3) {
		cerr << "Error: estimateStoryDisp(): cmat should be 3 by 3.\n";
		return -1;
	}
	if (!(dvec.rows >= 4 && dvec.cols == 1) && !(dvec.rows == 1 && dvec.cols >= 4)) {
		cerr << "Error: estimateStoryDisp(): dvec should be (>=4) by 1 or 1 by (>=4).\n";
		return -1;
	}
	if (rvec.rows == 3 && rvec.cols == 3)
	{
		cv::Mat rvec3(3, 1, rvec.type()); 
		cv::Rodrigues(rvec, rvec3); 
		rvec = rvec3.clone(); 
	}
	if (rvec.rows * rvec.cols != 3) 
	{
		cerr << "Error: estimateStoryDisp(): rvec should be 3x1 or 1x3 or 3x3.\n";
		return -1;
	}
	if (tvec.rows * tvec.cols != 3) {
		cerr << "Error: estimateStoryDisp(): tvec should be 3x1 or 1x3.\n";
		return -1;
	}
	// Make sure matrices are one-column.
	//if (refImgPointsMat.rows == 1 && refImgPointsMat.cols >= 1) refImgPointsMat = refImgPointsMat.t(); 
	//if (trkImgPointsMat.rows == 1 && trkImgPointsMat.cols >= 1) trkImgPointsMat = trkImgPointsMat.t();
	if (refObjPointsMat.rows == 1 && refObjPointsMat.cols >= 1) refObjPointsMat = refObjPointsMat.t();
	if (trkObjPointsMat.rows == 1 && trkObjPointsMat.cols >= 1) trkObjPointsMat = trkObjPointsMat.t();
	if (newRefImgPointsMat.rows == 1 && newRefImgPointsMat.cols >= 1) newRefImgPointsMat = newRefImgPointsMat.t();
	if (newTrkImgPointsMat.rows == 1 && newTrkImgPointsMat.cols >= 1) newTrkImgPointsMat = newTrkImgPointsMat.t();
	//refImgPointsMat.convertTo(refImgPointsMat, CV_32FC2);
	//trkImgPointsMat.convertTo(trkImgPointsMat, CV_32FC2);
	refObjPointsMat.convertTo(refObjPointsMat, CV_64FC3);
	trkObjPointsMat.convertTo(trkObjPointsMat, CV_64FC3);
	newRefImgPointsMat.convertTo(newRefImgPointsMat, CV_32FC2);
	newTrkImgPointsMat.convertTo(newTrkImgPointsMat, CV_32FC2);
	
	// Declaration
	cv::Mat newRvec, newTvec;

	// Initial guess
	camRot = cv::Mat::zeros(3, 1, CV_64F); 
	newDisp = cv::Mat::zeros(3, 1, CV_64F);

	// We are trying to find newRvec and newDisp, so that 
	//  after calling 
	//   calcStoryDispNewObjPoints(trkObjPointsMat, tortionCenter, newDisp, newTrkObjPoints); 
	//   cv::projectPoints(refObjPointsMat, newRvec, tvec, cmat, dvec, newProjRefImgPoints);
	//   cv::projectPoints(newTrkObjPoints, newRvec, tvec, cmat, dvec, newProjTrkImgPoints);
	//  newProjRefImgPoints - newRefImgPoints --> 0
	//  newProjTrkImgPoints - newTrkImgPoints --> 0

	// estimate a proper increment size ( 0.001 times longest distances among ref/trk points
	double maxDist = 0.0, dinc;
	for (int i = 0; i < refObjPointsMat.rows; i++) {
		for (int j = 0; j < trkObjPointsMat.rows; j++) {
			cv::Point3d p1 = refObjPointsMat.at<cv::Point3d>(i, 0);
			cv::Point3d p2 = trkObjPointsMat.at<cv::Point3d>(j, 0); 
			cv::Point3d pd = p1 - p2; 
			double dist = sqrt(pd.x * pd.x + pd.y * pd.y + pd.z * pd.z); 
			if (dist > maxDist) maxDist = dist;
		}
	}
	dinc = 0.001 * maxDist; 
	double dx[6] = { dinc, dinc, dinc, .01, .01, .01 }; // dx[3:5] is 0.01 radians

	// form jacobi matrix (Jij = - dyi / dxj) 
	// Set x0 (initial guess) 
	cv::Mat x0(6, 1, CV_64F), x1(6, 1, CV_64F);
	cv::Mat y0, y1;
	cv::Mat jacobi; 
	x0.at<double>(0, 0) = newDisp.at<double>(0, 0);
	x0.at<double>(1, 0) = newDisp.at<double>(1, 0);
	x0.at<double>(2, 0) = newDisp.at<double>(2, 0);
	x0.at<double>(3, 0) = camRot.at<double>(0, 0);
	x0.at<double>(4, 0) = camRot.at<double>(1, 0);
	x0.at<double>(5, 0) = camRot.at<double>(2, 0);
	// get y0
	costFuncStoryDispV4(cmat, dvec, rvec, tvec, 
		refObjPointsMat,
		trkObjPointsMat,
		newRefImgPointsMat,
		newTrkImgPointsMat, 
		tortionCenter, 
		newProjRefImgPointsMat, // new projected reference image points (fixed with camera), cv::Mat(N1, 1, CV_32FC2)
		newProjTrkImgPointsMat, // new projected tracking image points (not fixed with camera), cv::Mat(N2, 1, CV_32FC2)
		newTrkObjPoints, // new traking object points (not fixed with camera), cv::Mat(N2, 1, CV_64FC3)
		x0, y0);
	for (int j = 0; j < x0.rows; j++)
	{
		// set x1
		x1 = x0.clone();
		x1.at<double>(j, 0) += dx[j];
		// get y1
		costFuncStoryDispV4(cmat, dvec, rvec, tvec, 
			refObjPointsMat,
			trkObjPointsMat,
			newRefImgPointsMat,
			newTrkImgPointsMat, 
			tortionCenter, 
			newProjRefImgPointsMat, // new projected reference image points (fixed with camera), cv::Mat(N1, 1, CV_32FC2)
			newProjTrkImgPointsMat, // new projected tracking image points (not fixed with camera), cv::Mat(N2, 1, CV_32FC2)
			newTrkObjPoints, // new traking object points (not fixed with camera), cv::Mat(N2, 1, CV_64FC3)
			x1, y1);
		if (debug) cout << "x0: \n" << x0 << endl;
		if (debug) cout << "x1: \n" << x1 << endl;
		if (debug) cout << "y0: \n" << y0 << endl;
		if (debug) cout << "y1: \n" << y1 << endl;
		cv::Mat dy = y1 - y0;
		// set jacobi
		if (j == 0) jacobi = cv::Mat::zeros(y1.rows, x1.rows, CV_64F); 
		for (int i = 0; i < y1.rows; i++)
			jacobi.at<double>(i, j) = -dy.at<double>(i, 0) / dx[j];
	}
	if (debug) cout << "Jacobi(J):\n" << jacobi << endl;
	cv::Mat jtjijt = (jacobi.t() * jacobi).inv() * jacobi.t();

	x1 = x0.clone();
	y1 = y0.clone();
	cv::Mat x2, y2, dy;
	double dnorm, tol = 1e-6; 
	int itr, maxItr = 20; 
	for (itr = 0; itr < maxItr; itr++)
	{
		// adjust x1 --> x2
		x2 = x1 + jtjijt * y1;
		// get new y2
		costFuncStoryDispV4(cmat, dvec, rvec, tvec,
			refObjPointsMat,
			trkObjPointsMat,
			newRefImgPointsMat,
			newTrkImgPointsMat, 
			tortionCenter, 
			newProjRefImgPointsMat, // new projected reference image points (fixed with camera), cv::Mat(N1, 1, CV_32FC2)
			newProjTrkImgPointsMat, // new projected tracking image points (not fixed with camera), cv::Mat(N2, 1, CV_32FC2)
			newTrkObjPoints, // new traking object points (not fixed with camera), cv::Mat(N2, 1, CV_64FC3)
			x2, y2);
		if (debug) cout << "Iteration: " << itr << " y: " << y2 << endl;
		// update and check 
		dy = y2 - y1; 
		cv::Mat dyNorm = dy.t() * dy; 
		dnorm = sqrt(dyNorm.at<double>(0, 0));
		if (dnorm / dy.rows < tol) break;
		x1 = x2; 
		y1 = y2; 
	}
	if (itr >= maxItr)
	{
		printf("Warning: Story Displacement estimation does not converge. "
			"After %d iterations residual norm is %f.\n", itr, dnorm); 
	}

	newDisp.at<double>(0, 0)  = x2.at<double>(0, 0);
	newDisp.at<double>(1, 0)  = x2.at<double>(1, 0);
	newDisp.at<double>(2, 0)  = x2.at<double>(2, 0);
	camRot.at<double>(0, 0)   = x2.at<double>(3, 0);
	camRot.at<double>(1, 0)   = x2.at<double>(4, 0);
	camRot.at<double>(2, 0)   = x2.at<double>(5, 0);

	updatedRvecTvecByCameraRotation(rvec, tvec, camRot, newRvec, newTvec); 
	 
	newProjRefImgPoints = newRefImgPointsMat.clone();  
	newProjTrkImgPoints = newTrkImgPointsMat.clone();
	for (int i = 0; i < newProjRefImgPoints.rows; i++)
	{
		newProjRefImgPoints.at<cv::Point2f>(i, 0) += 
			 cv::Point2f((float)y2.at<double>(i * 2, 0), (float)y2.at<double>(i * 2 + 1, 0));
	}
	for (int i = 0; i < newProjTrkImgPoints.rows; i++)
	{
		newProjTrkImgPoints.at<cv::Point2f>(i, 0) +=
			 cv::Point2f((float)y2.at<double>(newProjRefImgPoints.rows * 2 + i * 2, 0), (float)y2.at<double>(newProjRefImgPoints.rows * 2 + i * 2 + 1, 0));
	}
	//

	if (debug) cout << "Projection reference points: \n" << newProjRefImgPoints << endl;
	if (debug) cout << "Projection tracking points: \n" << newProjTrkImgPoints << endl;

	return 0;
}

/*!
	\brief Function estimateStoryDispV4() estimates story displacement function of story displacement
	\param cmat					camera matrix of intrinsic parameters, cv::Mat(3, 3, CV_32F)
	\param dvec					camera distortion vector of intrinsic parameters, cv::Mat(1, 4 or 5 or 8, CV_32F)
	\param rvec					rotational vector of camera extrinsic parameters, cv::Mat(3, 1, CV_64F)
	\param tvec					translational vector of camera extrinsic parameters, cv::Mat(3, 1, CV_64F)
	\param refImgPoints         image points of reference points (supposed to be fixed on camera story where camera is fixed on), cv::Mat(N1, 1, CV_64FC3) or vector<cv::Point3d>
	\param trkImgPoints         image points of tracking points (moving points on measurement story), cv::Mat(N1, 1, CV_64FC3) or vector<cv::Point3d>
	\param refObjPoints         reference points in world coord. Can be vector<cv::Point3d> or cv::Mat(N1,1,CV_64FC3)
	\param trkObjPoints         tracking points in world coord. Can be vector<cv::Point3d> or cv::Mat(N2,1,CV_64FC3)
	\param newRefImgPoints      moved reference points in image coord. vector<cv::Point2f> or cv::Mat(N1,1,CV_32FC2)
	\param newTrkImgPoints      moved tracking points in image coord. vector<cv::Point2f> or cv::Mat(N2,1,CV_32FC2)
	\param tortionCenter        tortional center point in world coord. cv::Point3d.
	\param camRot               camera rotational movement (3, 1, CV_64F)
	\param newDisp              displacement of story (ux, uy, tortion) (3, 1, CV_64F)
	\param newTrkObjPoints      moved tracking points in world coord. cv::Mat(N2,1,CV_64FC3)
	\param newProjRefImgPoints  new projection reference points in image coord (N1, 1, CV_32FC2)
	\param newProjTrkImgPoints  new projection tracking points in image coord (N2, 1, CV_32FC2)
*/
int estimateStoryDispV4(
	cv::Mat cmat,			         // camera matrix 
	cv::Mat dvec,			         // distortion vector
	cv::Mat rvec,			         // rotational vector (3, 1, CV_64F)
	cv::Mat tvec,			         // translational vector (3, 1, CV_64F)
	cv::InputArray refImgPoints,     // reference points in image coord. Can be vector<cv::Point2f> or cv::Mat(N,1,CV_32FC2)
	cv::InputArray trkImgPoints,     // tracking points in image coord. Can be vector<cv::Point2f> or cv::Mat(N,1,CV_32FC2)
	cv::InputArray refObjPoints,     // reference points in world coord. Can be vector<cv::Point3d> or cv::Mat(N,1,CV_64FC3)
	cv::InputArray trkObjPoints,     // tracking points in world coord. Can be vector<cv::Point3d> or cv::Mat(N,1,CV_64FC3)
	cv::InputArray newRefImgPoints,  // moved reference points in image coord. vector<cv::Point2f> or cv::Mat(N,1,CV_32FC2)
	cv::InputArray newTrkImgPoints,  // moved tracking points in image coord. vector<cv::Point2f> or cv::Mat(N,1,CV_32FC2)
	cv::Point3d    tortionCenter,    // tortional center point in world coord. 
	cv::Mat & camRot,                // camera rotational movement (3, 1, CV_64F)
	cv::Mat & newDisp,               // displacement of story (ux, uy, tortion) (3, 1, CV_64F) 
	cv::Mat & newTrkObjPoints,       // moved tracking points in world coord. cv::Mat(N,1,CV_64FC3)
	cv::Mat & newProjRefImgPoints,   // new projection reference points in image coord.
	cv::Mat & newProjTrkImgPoints    // new projection tracking points in image coord. 
)
{
	bool debug = false; 
	// Estimate the offsets of zero displacement and zero cam-rotation by givingn initial image points. 
	cv::Mat camRotZeroOffset(3, 1, CV_64F); // Offset of zero camera rotation
	cv::Mat newDispZeroOffset(3, 1, CV_64F); // Offset of zero story displacement

	estimateStoryDispWithoutReturningZeroV4(cmat, dvec, rvec, tvec,
		refObjPoints, trkObjPoints,
		refImgPoints, trkImgPoints,
		tortionCenter,
		camRotZeroOffset,
		newDispZeroOffset,
		newTrkObjPoints,
		newProjRefImgPoints,
		newProjTrkImgPoints); 
	if (debug) cout << "camRotZeroOffset: \n" << camRotZeroOffset << endl;
	if (debug) cout << "newDispZeroOffset.:\n" << newDispZeroOffset << endl;

	estimateStoryDispWithoutReturningZeroV4(cmat, dvec, rvec, tvec,
		//refImgPoints, trkImgPoints, 
		refObjPoints, trkObjPoints,
		newRefImgPoints, newTrkImgPoints,
		tortionCenter,
		camRot,
		newDisp,
		newTrkObjPoints,
		newProjRefImgPoints,
		newProjTrkImgPoints);

	camRot -= camRotZeroOffset;
	newDisp -= newDispZeroOffset; 

	if (debug) cout << "camRot: \n" << camRot << endl;
	if (debug) cout << "newDisp.:\n" << newDisp << endl;

	return 0; 
}


int test_calcStoryDispNewObjPointsV4()
{
	bool debug = false; 
	cv::Mat objPoints(1, 1, CV_64FC3);
	objPoints.at<cv::Point3d>(0, 0).x = 4.0;
	objPoints.at<cv::Point3d>(0, 0).y = 2.0;
	objPoints.at<cv::Point3d>(0, 0).z = 0.0;
	cv::Point3d center(3, 1, 0);
	cv::Mat u(3, 1, CV_64F);
	u.at<double>(0, 0) = 0.0;
	u.at<double>(1, 0) = 0.0;
	u.at<double>(2, 0) = 1.0;
	if (debug == true) cout << "Old point:\n" << objPoints << endl;
	if (debug == true) cout << "Center:\n" << center << endl;
	if (debug == true) cout << "Disp: \n" << u << endl;
	calcStoryDispNewObjPoints(objPoints, center, u, objPoints);
	if (debug == true) cout << "New point:\n" << objPoints << endl;
	return 0;
}

int test_estimateStoryDispV4() 
{
	bool debug = false;
	cv::Mat cmat, dvec, rvec, tvec;
	cv::Mat objPoints, imgPoints;
	cv::Mat refObjPoints, refImgPoints, trkObjPoints, trkImgPoints; 
	cv::Mat camRot, newRvec, newTvec, newDisp, refProjError, trkProjError;
	cv::Mat newRefImgPoints, newTrkImgPoints, newTrkObjPoints; 
	cv::Mat newProjRefImgPoints, newProjTrkImgPoints; 
	cv::Point3d tortionCenter;

	vector<double> rvvec, tvvec;
	string fname1 = "C:/ExpDataSamples/2006RcBrickWall/Videos/Analysis_20161013_1800gal_C1_Calibration/C1_calib.xml";
	string fname2 = "C:/ExpDataSamples/2006RcBrickWall/Videos/Analysis_20161013_1800gal_C1_Calibration/IMG0000_10_objPoints.xml";
	string fname3 = "C:/ExpDataSamples/2006RcBrickWall/Videos/Analysis_20161013_1800gal_C1_Calibration/IMG0000_10_pickedPoints.xml";
	cv::FileStorage ifs(fname1, cv::FileStorage::READ);
	if (ifs.isOpened() == false)
	{
		cerr << "Cannot open " << fname1 << endl;
		return -1;
	}
	ifs["cameraMatrix"] >> cmat;
	ifs["distortionVector"] >> dvec;
	ifs["rvec"] >> rvvec; rvec = cv::Mat(rvvec);
	ifs["tvec"] >> tvvec; tvec = cv::Mat(tvvec);
	if (debug == true) cout << "Cmat:\n" << cmat << endl;
	if (debug == true) cout << "Dvec:\n" << dvec << endl;
	if (debug == true) cout << "Rvec:\n" << rvec << endl;
	if (debug == true) cout << "Tvec:\n" << tvec << endl;
	ifs.release();

	ifs.open(fname2, cv::FileStorage::READ);
	ifs["Points3dHistoryData"] >> objPoints;
	ifs.release();
	ifs.open(fname3, cv::FileStorage::READ);
	ifs["Points2fHistoryData"] >> imgPoints;
	ifs.release();
	if (debug == true) cout << "All obj points:\n" << objPoints << endl;
	if (debug == true) cout << "All img points:\n" << imgPoints << endl;

	objPoints = objPoints.t(); 
	imgPoints = imgPoints.t();

	objPoints(cv::Rect(0, 0, 1, 5)).copyTo(trkObjPoints);
	imgPoints(cv::Rect(0, 0, 1, 5)).copyTo(trkImgPoints);
	objPoints(cv::Rect(0, 4, 1, 6)).copyTo(refObjPoints);
	imgPoints(cv::Rect(0, 4, 1, 6)).copyTo(refImgPoints);

	if (debug == true) cout << "Tracking object points:\n" << trkObjPoints << endl;
	if (debug == true) cout << "Tracking image points:\n" << trkImgPoints << endl;
	if (debug == true) cout << "Reference object points:\n" << refObjPoints << endl;
	if (debug == true) cout << "Reference image points:\n" << refImgPoints << endl;

	newRefImgPoints = refImgPoints.clone();
	newTrkImgPoints = trkImgPoints.clone(); 

	// artificial cam move
	for (int i = 0; i < newRefImgPoints.rows; i++) newRefImgPoints.at<cv::Point2f>(i, 0) += cv::Point2f(10.f, 0.f); 
	for (int i = 0; i < newTrkImgPoints.rows; i++) newTrkImgPoints.at<cv::Point2f>(i, 0) += cv::Point2f(10.f, 0.f);

	//newRefImgPoints.at<cv::Point2f>(0, 0).x += (float) -75.982666 / 100.0f; 
	//newRefImgPoints.at<cv::Point2f>(1, 0).x += (float) 91.7114257 / 100.0f; 
	//newRefImgPoints.at<cv::Point2f>(2, 0).x += (float) 119.210815 / 100.0f; 
	//newRefImgPoints.at<cv::Point2f>(3, 0).x += (float) -61.755371 / 100.0f; 
	//newRefImgPoints.at<cv::Point2f>(4, 0).x += (float) -99.975585 / 100.0f; 
	//newRefImgPoints.at<cv::Point2f>(5, 0).x += (float) -122.80273 / 100.0f; 
	//newTrkImgPoints.at<cv::Point2f>(0, 0).x += (float) -93.753051 / 100.0f; 
	//newTrkImgPoints.at<cv::Point2f>(1, 0).x += (float) -71.945190 / 100.0f; 
	//newTrkImgPoints.at<cv::Point2f>(2, 0).x += (float) 72.9858398 / 100.0f; 
	//newTrkImgPoints.at<cv::Point2f>(3, 0).x += (float) 55.4199218 / 100.0f; 
	//newRefImgPoints.at<cv::Point2f>(0, 0).y += (float) -1769.60144 / 100.f; 
	//newRefImgPoints.at<cv::Point2f>(1, 0).y += (float) -1818.43872 / 100.f; 
	//newRefImgPoints.at<cv::Point2f>(2, 0).y += (float) -1846.88110 / 100.f; 
	//newRefImgPoints.at<cv::Point2f>(3, 0).y += (float) -1776.86157 / 100.f; 
	//newRefImgPoints.at<cv::Point2f>(4, 0).y += (float) -1795.70312 / 100.f; 
	//newRefImgPoints.at<cv::Point2f>(5, 0).y += (float) -1841.13159 / 100.f; 
	//newTrkImgPoints.at<cv::Point2f>(0, 0).y += (float) -1856.11877 / 100.f; 
	//newTrkImgPoints.at<cv::Point2f>(1, 0).y += (float) -1788.57727 / 100.f; 
	//newTrkImgPoints.at<cv::Point2f>(2, 0).y += (float) -1786.92627 / 100.f; 
	//newTrkImgPoints.at<cv::Point2f>(3, 0).y += (float) -1746.19140 / 100.f; 

	estimateStoryDispV4(cmat, dvec, rvec, tvec,
		refImgPoints,
		trkImgPoints,
		refObjPoints,
		trkObjPoints,
		newRefImgPoints,
		newTrkImgPoints,
		tortionCenter,
		camRot,
		newDisp,
		newTrkObjPoints, 
		newProjRefImgPoints,
		newProjTrkImgPoints);

	updatedRvecTvecByCameraRotation(rvec, tvec, camRot, newRvec, newTvec);
	
	cout << "New disp: \n" << newDisp << endl;
	cout << "Camera rot.:\n" << camRot << endl;
	cout << "New rvec:\n" << newRvec << endl;
	cout << "Reference points projection points: \n" << newRefImgPoints << endl;
	cout << "Tracking points projection points: \n" << newTrkImgPoints << endl;

	return 0;
}
