#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <cmath>
#include <vector>
#include <string>
#include <ctime>
#include <chrono>
#include <thread>

using namespace std;

#include <opencv2/opencv.hpp>


#include "improCalib.h"

cv::Mat r44_64F(cv::Mat rvec, cv::Mat tvec)
{
    bool debug = true;
    int check = checkAndConvertExtrinsic(rvec, tvec);
    if (check != 0) return cv::Mat();
    cv::Mat r44 = cv::Mat::eye(4, 4, CV_64F);
    cv::Mat r33(r44(cv::Rect(0, 0, 3, 3)));
    if (tvec.cols == 3 && tvec.rows == 1) tvec = tvec.t();
    cv::Rodrigues(rvec, r33);
    tvec.copyTo(r44(cv::Rect(3, 0, 1, 3)));
    if (debug) {
        cout << "Converted " << rvec << "\n and " << tvec << "\n to \n" << r44 << "\n";
        cout.flush();
    }
    return r44;
}

cv::Mat r44_32F(cv::Mat rvec, cv::Mat tvec)
{
    cv::Mat m;
    r44_64F(rvec, tvec).convertTo(m, CV_32F);
    return m;
}

cv::Mat camPos_64F(cv::Mat rvec, cv::Mat tvec)
{
    int check = checkAndConvertExtrinsic(rvec, tvec);
    if (check != 0) return cv::Mat();
    return r44_64F(rvec, tvec).inv()(cv::Rect(3, 0, 1, 3)).t();
}

cv::Mat camPos_32F(cv::Mat rvec, cv::Mat tvec)
{
    int check = checkAndConvertExtrinsic(rvec, tvec);
    if (check != 0) return cv::Mat();
    return r44_32F(rvec, tvec).inv()(cv::Rect(3, 0, 1, 3)).t();
}


double calibrateCameraSingleImage(
        const cv::Mat & objPoints, //!< global (world) coord. of known points. (1, N, CV_32FC3)
        const cv::Mat & imgPoints,
        const std::vector<cv::Mat> & colinearPoints,
             cv::Size   imgSize,
              cv::Mat & cmat,
              cv::Mat & dvec,
              cv::Mat & rvec,
              cv::Mat & tvec,
              cv::Mat & stdDeviationsIntrinsics,
              cv::Mat & stdDeviationsExtrinsics,
              cv::Mat & perViewErrors,
                  int   flags,
       cv::TermCriteria criteria)
{
    double rms = 0.0;
    vector<vector<cv::Point3f>> objPointsVec(1);
    vector<vector<cv::Point2f>> imgPointsVec(1);
    bool debug = true;
    int nObjPoints = objPoints.cols;

    // conventional opencv calibration
    objPointsVec[0] = vector<cv::Point3f>(objPoints.cols * objPoints.rows);
    for (int i = 0; i < objPoints.cols * objPoints.rows; i++) {
        objPointsVec[0][i] = objPoints.at<cv::Point3f>(i);
    }
    imgPointsVec[0] = vector<cv::Point2f>(imgPoints.cols * imgPoints.rows);
    for (int i = 0; i < imgPoints.cols * imgPoints.rows; i++) {
        imgPointsVec[0][i] = imgPoints.at<cv::Point2f>(i);
    }

    // set initial guess of cmat
    if ((flags & cv::CALIB_USE_INTRINSIC_GUESS) == 0)
    {
        rvec = cv::Mat::zeros(3, 1, CV_64F);
        tvec = cv::Mat::zeros(3, 1, CV_64F);
        cmat = cv::Mat::eye(3, 3, CV_64F);
        cmat.at<double>(0, 0) = imgSize.width;
        cmat.at<double>(1, 1) = imgSize.width;
        cmat.at<double>(0, 2) = (imgSize.width - 1) * .5f;
        cmat.at<double>(1, 2) = (imgSize.height - 1) * .5f;
        cmat.at<double>(2, 2) = 1;
        dvec = cv::Mat::zeros(1, 12, CV_32F);
        flags = flags | cv::CALIB_USE_INTRINSIC_GUESS;
        cout << "Gave a cmat initial guess for camera calibration: " << cmat << "\n";
    }

    // if flag
    if (flags == 0) {
        cout << "Warning: You gave calibration flags which are all zeros.\n";
        cout << "         Likely you could get divergence from camera calibration.\n";
        cout.flush();
//        return 0.0;
    }

//    dvec = dvec.t();
    try {
        rms = cv::calibrateCamera(objPointsVec,
                                  imgPointsVec,
                                  imgSize,
                                  cmat,
                                  dvec,
                                  rvec,
                                  tvec ,
                                  stdDeviationsIntrinsics,
                                  stdDeviationsExtrinsics,
                                  perViewErrors,
                                  flags,
                                  criteria
                                  );
        if (debug) {
            cout << "Calibrationf flags is " << flags << "\n";
            cout << "Cmat: (type: " << cmat.type() << ")\n" << cmat << "\n";
            cout << "Dvec: (type: " << cmat.type() << ")\n" << dvec << "\n";
            cout << "Rvec: (type: " << cmat.type() << ")\n" << rvec << "\n";
            cout << "Tvec: (type: " << cmat.type() << ")\n" << tvec << "\n";
            cout << "stdDeviationsIntrinsics: (type: " << cmat.type() << ")\n" << stdDeviationsIntrinsics << "\n";
            cout << "stdDeviationsExtrinsics: (type: " << cmat.type() << ")\n" << stdDeviationsExtrinsics << "\n";
            cout << "perViewErrors: (type: " << cmat.type() << ")\n" << perViewErrors << "\n";
            cout.flush();
        }
    } catch ( ... ) {
        cerr << "Error: Failed to do calibration.\n";
        cerr << "  Calibrationf flags is " << flags << "\n";
        cerr << "  Change the flags and try again.\n";
        cerr.flush();
    }

    int nColinears = (int) colinearPoints.size();
    if (colinearPoints.size() <= 0) return rms;
    int nColinearPoints = 0;

    // start adjusting parameters so that colinear points are also colinear undistorted.
    vector<vector<cv::Point3f>> appendObjPointsVec(objPointsVec);
    vector<vector<cv::Point2f>> appendImgPointsVec(imgPointsVec);
    for (int i = 0; i < nColinears; i++)
        nColinearPoints += colinearPoints[i].cols * colinearPoints[i].rows;
    appendObjPointsVec[0].resize(nObjPoints + nColinearPoints);
    appendImgPointsVec[0].resize(nObjPoints + nColinearPoints);

    // according to calibrated result (cmat and dvec), and the colinear points (xi)
    // estimate their global coordinates (xw), (and the projected xw should be different from xi)
    float avgDepth = 0;
    cv::Mat objPointMat = objPoints.reshape(1, objPoints.cols * objPoints.rows).t(); // A 3xn matrix, making points column vectors.
    cv::Mat R3; cv::Rodrigues(rvec, R3);
    if (debug) { cout << "R3: \n" << R3 << "\n"; cout.flush(); }
    cv::Mat Tvec(3, 1, CV_64F, tvec.data);
    if (debug) { cout << "Tvec:\n" << Tvec << "\n"; cout.flush(); }
    for (int i = 0; i < (int) objPointMat.cols; i++) {
        cv::Mat pInCam = objPointMat(cv::Rect(i, 0, 1, 3)).clone();
        pInCam.convertTo(pInCam, CV_64F);
        if (debug) { cout << "pInCam:\n" << pInCam << "\n"; cout.flush(); }
        pInCam = R3 * pInCam + Tvec;
        if (debug) { cout << "pInCam:\n" << pInCam << "\n"; cout.flush(); }
        pInCam.convertTo(pInCam, CV_32F);
        if (debug) { cout << "pInCam:\n" << pInCam << "\n"; cout.flush(); }
        pInCam.copyTo(objPointMat(cv::Rect(i, 0, 1, 3)));
    }
    if (debug) {
        cout << "Object points in camera view: \n" << objPointMat << "\n";
        cout.flush();
    }
    avgDepth = 0.0f;
    for (int i = 0; i < (int) objPointMat.cols; i++) {
        avgDepth += objPointMat.at<float>(2, i) / nObjPoints;
    }
    // undistort colinear points
    std::vector<cv::Mat> undistortColinearPoints(nColinears);
    int nPointsAppended = 0;
    for (int i = 0; i < nColinears; i++) {
        int nPointsThisColine = colinearPoints[i].cols;
        // undistort points (to normalized coordinate)
        cv::undistortPoints(colinearPoints[i], undistortColinearPoints[i],
                            cmat, dvec);
        if (debug) {
            cout << "Colinear " << i + 1 << " is: \n" << colinearPoints[i] << "\n";
            cout << "Undistorted colinear " << i + 1 << " is: \n" << undistortColinearPoints[i] << "\n";
        }
        // find linear projection image points so that they are colinear in undistorted image
        cv::Mat colinearData = cv::Mat(nPointsThisColine, 2, CV_32F, undistortColinearPoints[i].data).clone();
        if (debug) {
            cout << "Undistorted colinear data: \n" << colinearData << "\n";
            cout.flush();
        }
        cv::PCA pca(colinearData, cv::Mat(), cv::PCA::DATA_AS_ROW);
        colinearData = pca.project(colinearData);
        colinearData = pca.backProject(colinearData);
        if (debug) {
            cout << "Undistorted colinear data backprojected: \n" << colinearData << "\n";
            cout.flush();
        }
        // guess their global (world) coordinates
        cv::Mat colinearGuessGlobal(3, nPointsThisColine, CV_64F);
        for (int j = 0; j < nPointsThisColine; j++) {
            colinearGuessGlobal.at<double>(0, j) = (double) colinearData.at<float>(j, 0) * avgDepth;
            colinearGuessGlobal.at<double>(1, j) = (double) colinearData.at<float>(j, 1) * avgDepth;
            colinearGuessGlobal.at<double>(2, j) = (double) avgDepth;
        }
        cv::Mat Tvecs; cv::repeat(Tvec, 1, nPointsThisColine, Tvecs);
        colinearGuessGlobal = R3.inv() * (colinearGuessGlobal - Tvecs);
        if (debug) {
            cout << "Colinear guessed global coord.: \n" << colinearGuessGlobal << "\n";
        }
        // append guessed global coord to new object points
        for (int j = 0; j < nPointsThisColine; j++) {
            float xw = (float) colinearGuessGlobal.at<double>(0, j);
            float yw = (float) colinearGuessGlobal.at<double>(1, j);
            float zw = (float) colinearGuessGlobal.at<double>(2, j);
            appendObjPointsVec[0][nObjPoints + nPointsAppended] = cv::Point3f(xw, yw, zw);
            appendImgPointsVec[0][nObjPoints + nPointsAppended] = colinearPoints[i].at<cv::Point2f>(j);
            nPointsAppended++;
        }
    }
    if (debug) {
        cout << "Appended obj points: \n" << appendObjPointsVec[0] << "\n";
        cout << "Appended img points: \n" << appendImgPointsVec[0] << "\n";
    }
    cout.flush();
    // try to calibrate again with the appended points
    try {
        rms = cv::calibrateCamera(appendObjPointsVec,
                                  appendImgPointsVec,
                                  imgSize,
                                  cmat,
                                  dvec,
                                  rvec,
                                  tvec ,
                                  stdDeviationsIntrinsics,
                                  stdDeviationsExtrinsics,
                                  perViewErrors,
                                  flags,
                                  criteria
                                  );
        if (debug) {
            cout << "Calibrationf flags is " << flags << "\n";
            cout << "Cmat: (type: " << cmat.type() << ")\n" << cmat << "\n";
            cout << "Dvec: (type: " << cmat.type() << ")\n" << dvec << "\n";
            cout << "Rvec: (type: " << cmat.type() << ")\n" << rvec << "\n";
            cout << "Tvec: (type: " << cmat.type() << ")\n" << tvec << "\n";
            cout << "stdDeviationsIntrinsics: (type: " << cmat.type() << ")\n" << stdDeviationsIntrinsics << "\n";
            cout << "stdDeviationsExtrinsics: (type: " << cmat.type() << ")\n" << stdDeviationsExtrinsics << "\n";
            cout << "perViewErrors: (type: " << cmat.type() << ")\n" << perViewErrors << "\n";
            cout.flush();
        }
    } catch ( ... ) {
        cerr << "Error: Failed to do calibration.\n";
        cerr << "  Calibrationf flags is " << flags << "\n";
        cerr << "  Change the flags and try again.\n";
        cerr.flush();
    }

    return rms;
}


int printIntrinsic(
        const cv::Mat & cmat,
        const cv::Mat & dvec,
        const cv::Mat & stdevIntrinsic,
        const cv::Mat & perViewErrors,
        std::ostream & sout)
{
    char buf[1000];
    sout << "# ==========================\n";
    sout << "# Camera calibration result:\n";
    snprintf(buf, 1000, "# Fx is %12.4f (stdev: %12.4f)\n",
         cmat.at<double>(0, 0), stdevIntrinsic.at<double>(0));
    sout << buf;
    snprintf(buf, 1000, "# Fy is %12.4f (stdev: %12.4f)\n",
         cmat.at<double>(1, 1), stdevIntrinsic.at<double>(1));
    sout << buf;
    snprintf(buf, 1000, "# Cx is %12.4f (stdev: %12.4f)\n",
         cmat.at<double>(0, 2), stdevIntrinsic.at<double>(2));
    sout << buf;
    snprintf(buf, 1000, "# Cy is %12.4f (stdev: %12.4f)\n",
         cmat.at<double>(1, 2), stdevIntrinsic.at<double>(3));
    sout << buf;
    for (int i = 0; i < dvec.cols * dvec.rows; i++) {
        snprintf(buf, 1000, "# Dist. coef. [%d] is %12.4f"
                            "(stdev: %12.4f)\n", i,
             dvec.at<double>(i), stdevIntrinsic.at<double>(i + 4));
        sout << buf;
    }
    sout << "# Calibration per-view errors: \n";
    for (int i = 0; i < perViewErrors.cols * perViewErrors.rows; i++)
    {
        snprintf(buf, 1000, "#   Per-view error [%d] is %12.4f\n",
                 i, perViewErrors.at<double>(i));
        sout << buf;

    }
    sout << "# End of calibration info.\n";
    return 0;
}

int calibrateCameraFromImageFileNames(
        std::vector<std::string> fullFileNames, //!< vector of full-path file names
        cv::Size patternSize, //!< pattern size (numbers of corners along x and y) of chessboard
        cv::Vec2f calibBlockSize, //!< width and height of a block on chessboard
        cv::Mat & cmat,
        cv::Mat & dvec,
        cv::Mat & rvecs,
        cv::Mat & tvecs,
        cv::Mat & stdevIntrinsic,
        cv::Mat & stdevExtrinsic,
        cv::Mat & perViewErrors,
        bool showCorners,
        bool showUndistorted,
        bool printInfo,
        int flags,
        cv::TermCriteria criteria
        )
{
    bool standAloneInteraction = false;
    // Step 1: Get the photos
    int numCalibPhotos;
    std::vector<std::string> filenames;
    std::vector<std::vector<cv::Vec2f>> imgPoints;
    std::vector<std::vector<cv::Vec3f>> objPoints;
    cv::Size imgSize;

    if (standAloneInteraction == true) {
        std::cout << "# How many photos do you have? \n";
        std::cin >> numCalibPhotos;
    } else {
        numCalibPhotos = (int) fullFileNames.size();
    }

    filenames.resize(numCalibPhotos);
    imgPoints.resize(numCalibPhotos);
    objPoints.resize(numCalibPhotos);

    if (standAloneInteraction == true) {
        std::cout << "# Enter full path file names one by one: \n";
        for (int i = 0; i < numCalibPhotos; i++) {
            std::cin >> filenames[i];
        }
    } else {
        for (int i = 0; i < numCalibPhotos; i++)
            filenames[i] = fullFileNames[i];
    }

    // ask how many corners do you have on the board
    int nRowsBlocks, nColsBlocks;
    float bSizeX, bSizeY;
    if (standAloneInteraction == true) {
        std::cout << "# How many corners (x by y) are in your"
                  << " calibrate board?\n";
        std::cin >> nRowsBlocks >> nColsBlocks;
        std::cout << "# What are the size (along x and y) "
                  << "of the block in the board?\n";
        std::cin >> bSizeX >> bSizeY;
    } else {
        nRowsBlocks = patternSize.width;
        nColsBlocks = patternSize.height;
        bSizeX = calibBlockSize[0];
        bSizeY = calibBlockSize[1];
    }

    for (int i = 0; i < numCalibPhotos; i++) {
        imgPoints[i].resize(nRowsBlocks * nColsBlocks);
        objPoints[i].resize(nRowsBlocks * nColsBlocks);
    }

    // Step 2: Read the next photo
    for (int i = 0; i < numCalibPhotos; i++) {
        // read image
        cv::Mat theImg;
        bool patternFound = false;
        // a while loop that read image and find corners until corners are found.
        while (true) {
            // a while loop that waits for a valid image
            while (true) {
                theImg = cv::imread(filenames[i],
                                            cv::IMREAD_GRAYSCALE);
                if (theImg.cols <= 0 || theImg.rows <= 0) {
                    for (int k = 0; k < 999; k++) printf("\b");
                    printf("# Warning from calibrateCameraFromImageFileNames()"
                           ". Waiting for a valid image from file %s."
                           " (clock %d)",
                           filenames[i].c_str(), (int) clock());
                    // wait if the file is not an image file
                    std::this_thread::sleep_for(std::chrono::milliseconds(1000) );
                } else {
                    break;
                }
            }

            imgSize.width = theImg.cols;
            imgSize.height = theImg.rows;
            if (printInfo == true) {
                printf("# Read image %s successfully which size is %d x %d.\n",
                       filenames[i].c_str(), theImg.cols, theImg.rows);
            }

            // Step 3: Get the image locations of the corners
            if (printInfo == true) printf("# Trying to find corners ...");
            patternFound =
            cv::findChessboardCorners(theImg,
                  cv::Size(nRowsBlocks, nColsBlocks),
                  imgPoints[i]);
            if (patternFound == true) {
                if (printInfo == true)
                    printf("# Corners found.\n");
            } else {
                printf("# Corners not found.\n");
                printf("# Warning: Cannot find corners in file %s. "
                       "Try to replace this image (or blur background by a photo editor."
                       "This program will wait.\n", filenames[i].c_str());
                continue;
            }
            cv::cornerSubPix(theImg, imgPoints[i],
                             cv::Size(3, 3),
                             cv::Size(-1, -1),
                             criteria);
            break;
        } // end of loop of reading image and finding corners until success.

        if (showCorners == true) {
            cv::Mat colorfulImg(theImg.rows, theImg.cols, CV_8UC3);
            cv::cvtColor(theImg, colorfulImg, cv::COLOR_GRAY2BGR);
            cv::drawChessboardCorners(colorfulImg,
                  cv::Size(nRowsBlocks, nColsBlocks),
                  imgPoints[i], patternFound);
            cv::imshow("Corners Found", colorfulImg);
            cv::waitKey(1000);
            cv::destroyWindow("Corners Found");
        }

        if (printInfo == true) {
            std::cout << "# Image points: \n";//
            for (int j = 0; j < nRowsBlocks * nColsBlocks; j++)
                printf("# %9.2f %9.2f\n",
                       imgPoints[i][j][0], imgPoints[i][j][1]);

        }

        // Step 4: Go back to Step 2 for next photo
    }

    // Step 5: Define the 3D coordinate of the corners (in chessboard coordinate)
    for (int i = 0; i < numCalibPhotos; i++) {
        for (int j = 0; j < nColsBlocks; j++) {
            for (int k = 0; k < nRowsBlocks; k++) {
                objPoints[i][k + j * nRowsBlocks] = cv::Vec3f(k * bSizeX, j * bSizeY, 0.0f);
            }
        }
    } // end of loop of each calibration photo

    // Step 6: Calibrate the camera by using OpenCV (cv::calibrateCamera())
//    cv::Mat cmat(3, 3, CV_64F), dvec(8, 1, CV_64F);
//    std::vector<cv::Mat> rvecs; //(numCalibPhotos, cv::Mat::zeros(3, 1, CV_64F));
//    std::vector<cv::Mat> tvecs; //(numCalibPhotos, cv::Mat::zeros(3, 1, CV_64F));
    cmat = cv::Mat::zeros(3, 3, CV_64F);
    dvec = cv::Mat::zeros(12, 1, CV_64F);

//    int flag = flags;
//               cv::CALIB_FIX_K2 |
//               cv::CALIB_FIX_K3 |
//               cv::CALIB_FIX_K4 |
//               cv::CALIB_FIX_K5 |
//               cv::CALIB_FIX_K6 ;

    if (printInfo == true) {
        printf("# Calibrating the camera ...");
    }
    cv::calibrateCamera(objPoints,
        imgPoints,
        imgSize,
        cmat, dvec, rvecs, tvecs,
        stdevIntrinsic,
        stdevExtrinsic,
        perViewErrors,
        flags, criteria);

    if (printInfo == true) {
        printf("# OK\n");
    }

    // Step 7: Print the parameters
    if (printInfo) {
        printIntrinsic(cmat, dvec, stdevIntrinsic, perViewErrors);
    } // end of if printInfo

    if (showUndistorted) {
        // Step 8: Undistortion each of the photos
        // Step 9: Compare the original photos and the undistorted photos
        for (int i = 0; i < numCalibPhotos; i++) {
            // Read the image again
            cv::Mat theImg = cv::imread(filenames[i]);
            // Undistort the image
            cv::Mat undImg;   //!< undistorted image
            cv::undistort(theImg, undImg, cmat, dvec);
            // Write the undistorted image to a JPG file
            cv::imwrite(filenames[i] + "_undistorted.JPG", undImg);
            // Show the images
            cv::imshow("Original image", theImg);
            cv::imshow("Undistorted image", undImg);
            cv::waitKey(0);
            cv::destroyWindow("Original image");
            cv::destroyWindow("Undistorted image");
        } // end of loop of each calibration photo
    }

    return 0;
}

/*! \brief Checks correctness of intrinsic parameters. Converts to correct dimension and format.
 * This function checks the correctness of intrinsic parameters including camera matrix (cmat),
 * and distortion coefficients (dvec).
 * The dvec is supposed to be a row vector. If it is a column vector,
 * it will be transposed.
 * All parameters should be CV_64F (double). If any one of them is 32-bit, it will be converted
 * to 64-bit.
 * \param cmat the camera matrix, in format of cv::Mat(3, 3, CV_64F)
 * \param dvec the distortion vector, in format of cv::Mat(1, nDistortCoefs, CV_64F).
 * \return 0 for success.
*/
int checkAndConvertIntrinsic(
        cv::Mat & cmat,
        cv::Mat & dvec)
{
    // check cmat
    if (cmat.cols != 3 || cmat.rows != 3) {
        std::cerr << "Error: cmat size is incorrect.\n";
        std::cerr << "  Should be 3x3 but it is " << cmat.rows << "x" << cmat.cols << "\n";
        return -1;
    }
    // check dvec
    if (dvec.rows > 1 && dvec.cols == 1) dvec = dvec.t();
    if (dvec.rows != 1 || dvec.cols <= 4) {
        std::cerr << "Error: dvec size is incorrect.\n";
        std::cerr << "  Should be 1xn (n>=4) but it is " << dvec.rows << "x" << dvec.cols << "\n";
        return -1;
    }
    // convert to 64F format
    if (cmat.type() != CV_64F) cmat.convertTo(cmat, CV_64F);
    if (dvec.type() != CV_64F) dvec.convertTo(dvec, CV_64F);
    return 0;
}

/*! \brief Checks correctness of extrinsic parameters. Converts to correct dimension and format.
 * This function checks the correctness of extrinsic parameters including rotational vector (rvec)
 * and translational vector (tvec).
 * The rvec, and tvec are supposed to be row vector. If any one of them is a column vector,
 * it will be transposed.
 * All parameters should be CV_64F (double). If any one of them is 32-bit, it will be converted
 * to 64-bit.
 * \param rvec the rotational vector, in format of cv::Mat(1, 3, CV_64F).
 * \param tvec the translational vector, in format of cv::Mat(1, 3, CV_64F).
 * \return 0 for success.
*/
int checkAndConvertExtrinsic(
        cv::Mat & rvec,
        cv::Mat & tvec)
{
    // check rvec
    if (rvec.cols == 3 && rvec.rows == 3) {
        cv::Rodrigues(rvec, rvec);
    }
    if (rvec.rows == 3 && rvec.cols == 1) {
        rvec = rvec.t();
    }
    if (rvec.rows != 1 || rvec.cols != 3) {
        std::cerr << "Error: rvec size is incorrect.\n";
        std::cerr << "   Should be 1x3 but it is " << rvec.rows << "x" << rvec.cols << "\n";
        return -1;
    }
    // check tvec
    if (tvec.rows == 3 && tvec.cols == 1) {
        tvec = tvec.t();
    }
    if (tvec.rows != 1 || tvec.cols != 3) {
        std::cerr << "Error: tvec size is incorrect.\n";
        std::cerr << "   Should be 1x3 but it is " << tvec.rows << "x" << tvec.cols << "\n";
        return -1;
    }
    // convert to 64F format
    if (rvec.type() != CV_64F) rvec.convertTo(rvec, CV_64F);
    if (tvec.type() != CV_64F) tvec.convertTo(tvec, CV_64F);
    return 0;
}


/*! \brief Checks correctness of camera parameters. Converts to correct dimension and format.
 * This function checks the correctness of camera parameters including camera matrix (cmat),
 * distortion coefficients (dvec), rotational vector (rvec), and translational vector (tvec).
 * The dvec, rvec, and tvec are supposed to be row vector. If any one of them is a column vector,
 * it will be transposed.
 * All parameters should be CV_64F (double). If any one of them is 32-bit, it will be converted
 * to 64-bit.
 * \param cmat the camera matrix, in format of cv::Mat(3, 3, CV_64F)
 * \param dvec the distortion vector, in format of cv::Mat(1, nDistortCoefs, CV_64F).
 * \param rvec the rotational vector, in format of cv::Mat(1, 3, CV_64F).
 * \param tvec the translational vector, in format of cv::Mat(1, 3, CV_64F).
 * \return 0 for success.
*/
int checkAndConvertCalibration(cv::Mat & cmat,
                               cv::Mat & dvec,
                               cv::Mat & rvec,
                               cv::Mat & tvec)
{
    int ret;
    int check1 = checkAndConvertIntrinsic(cmat, dvec);
    int check2 = checkAndConvertExtrinsic(rvec, tvec);
    if (check1 == 0 && check2 == 0)
        ret = 0;
    else
        ret = 1;
    return ret;
}


/*! \brief Saves camera parameters to a cv::FileStorage
 *
 * This function saves camera intrinsic and extrinsic parameters to an
 * cv::FileStorage.
 * The tags and formats are
 * <imageSize> cv::Size
 * <cameraMatrix> or <cmat> cv::Mat(3, 3, CV_64F)
 * <distortionVector> or <dvec> cv::Mat(1, nDistortCoefs, CV_64F)
 * <rvec> cv::Mat(1, 3, CV_64F)
 * <tvec> cv::Mat(1, 3, CV_64F)
 * <R44> or <r44> cv::Mat(4, 4, CV_64F)
 * <CamPosition> or <campos> cv::Mat(1, 3, CV_64F)
 * \param imgSize image size cv::Size(width, height)
 * \param cmat camera matrix cv::Mat(3, 3, CV_64F)
 * \param dvec distortion vector cv::Mat(1, nDistortCoefs, CV_64F)
 * \param rvec rotational vector cv::Mat(1, 3, CV_64F)
 * \param tvec translational vector cv::Mat(1, 3, CV_64F)
 * \param ofs output file storage
 * \return string("0") for success, or warning/error message otherwise.
 * \sa saveCalibrationToFile()
*/
std::string saveCalibrationToFileStorage(
        cv::Size imgSize,
        cv::Mat cmat,
        cv::Mat dvec,
        cv::Mat rvec,
        cv::Mat tvec,
        cv::FileStorage & ofs)
{
    std::string ret;
    // check parameters
    int check = checkAndConvertCalibration(cmat, dvec, rvec, tvec);
    if (check != 0) {
        ret.append("-1\n");
        ret.append("Error: Cannot save camera parameters because of something wrong.\n");
        ret.append("       Check error messsage and parameter format.\n");
        std::cerr << ret;
        return ret;
    }
    if (ofs.isOpened() == false) {
        ret.append("-1\n");
        ret.append("Error: Cannot save camera parameters because of something wrong.\n");
        ret.append("       Cannot file open storage.\n");
        std::cerr << ret;
    }
    // write image size
    ofs << "imageSize" << imgSize;
    ofs << "imgSize" << imgSize;
    // write camera matrix
    ofs << "cameraMatrix" << cmat;
    ofs << "cmat" << cmat;
    ofs << "fx" << cmat.at<double>(0, 0);
    ofs << "fy" << cmat.at<double>(1, 1);
    ofs << "cx" << cmat.at<double>(0, 2);
    ofs << "cy" << cmat.at<double>(1, 2);
    // write distortion vector
    ofs << "distortionVector" << dvec;
    ofs << "dvec" << dvec;
    // write rvec
    ofs << "rvec" << rvec;
    ofs << "tvec" << tvec;
    // write R44
    cv::Mat r44 = r44_64F(rvec, tvec);
    ofs << "R44" << r44;
    ofs << "r44" << r44;
    // write campos
    cv::Mat campos = r44_64F(rvec, tvec).inv()(cv::Rect(3, 0, 1, 3)).t();
    ofs << "CamPosition" << campos;
    ofs << "campos" << campos;
    ofs.release();
    return ("0");
}

/*! \brief Saves camera parameters to a file
 *
 * This function saves camera intrinsic and extrinsic parameters to a file.
 * The tags and formats are
 * <imageSize> cv::Size
 * <cameraMatrix> or <cmat> cv::Mat(3, 3, CV_64F)
 * <distortionVector> or <dvec> cv::Mat(1, nDistortCoefs, CV_64F)
 * <rvec> cv::Mat(1, 3, CV_64F)
 * <tvec> cv::Mat(1, 3, CV_64F)
 * <R44> or <r44> cv::Mat(4, 4, CV_64F)
 * <CamPosition> or <campos> cv::Mat(1, 3, CV_64F)
 * \param imgSize image size cv::Size(width, height)
 * \param cmat camera matrix cv::Mat(3, 3, CV_64F)
 * \param dvec distortion vector cv::Mat(1, nDistortCoefs, CV_64F)
 * \param rvec rotational vector cv::Mat(1, 3, CV_64F)
 * \param tvec translational vector cv::Mat(1, 3, CV_64F)
 * \param fname full file path to write
 * \return string("0") for success, or warning/error message otherwise.
 * \sa saveCalibrationToFileStorage()
*/
std::string saveCalibrationToFile(
        cv::Size imgSize,
        cv::Mat cmat,
        cv::Mat dvec,
        cv::Mat rvec,
        cv::Mat tvec,
        std::string fname)
{
    std::string ret;
    // check parameters
    int check = checkAndConvertCalibration(cmat, dvec, rvec, tvec);
    if (check != 0) {
        ret.append("-1\n");
        ret.append("Error: Cannot save camera parameters because of something wrong.\n");
        ret.append("       Check error messsage and parameter format.\n");
        std::cerr << ret;
        return ret;
    }
    // check file
    if (fname.length() <= 1) {
        ret.append("-1\n");
        ret.append("Error: Cannot save camera parameters because of something wrong.\n");
        ret.append("       Check file path: " + fname + "\n");
        std::cerr << ret;
        return ret;
    }
    // open file storage to save
    cv::FileStorage ofs(fname, cv::FileStorage::WRITE);
    if (ofs.isOpened() == false) {
        ret.append("-1\n");
        ret.append("Error: Cannot save camera parameters because of something wrong.\n");
        ret.append("       Cannot open: " + fname + " as a file storage.\n");
        std::cerr << ret;
    }
    ret = saveCalibrationToFileStorage(imgSize, cmat, dvec, rvec, tvec, ofs);
    if (ret.compare("0") != 0) {
        std::cerr << "Error: saveCalibrationToFile() receives " << ret
                  << " from saveCalibrationToFileStorage().\n";
    }

    return ret;
}


/*! \brief Loads camera parameters from a cv::FileStorage
 *
 * This function loads camera intrinsic and extrinsic parameters from an
 * cv::FileStorage.
 * The tags and formats are
 * <imageSize> cv::Size
 * <cameraMatrix> cv::Mat(3, 3, CV_64F)
 * <distortionVector> cv::Mat(1, nDistortCoefs, CV_64F)
 * <rvec> cv::Mat(1, 3, CV_64F)
 * <tvec> cv::Mat(1, 3, CV_64F)
 * <R44> cv::Mat(4, 4, CV_64F)
 * <CamPosition> cv::Mat(1, 3, CV_64F)
 * \param imgSize image size cv::Size(width, height)
 * \param cmat camera matrix cv::Mat(3, 3, CV_64F)
 * \param dvec distortion vector cv::Mat(1, nDistortCoefs, CV_64F)
 * \param rvec rotational vector cv::Mat(1, 3, CV_64F)
 * \param tvec translational vector cv::Mat(1, 3, CV_64F)
 * \param ifs input file storage
 * \return string("0") for success, or warning/error message otherwise.
 * \sa loadCalibrationFromFile()
*/
std::string loadCalibrationFromFileStorage(
        cv::Size & imgSize,
        cv::Mat & cmat,
        cv::Mat & dvec,
        cv::Mat & rvec,
        cv::Mat & tvec,
        cv::FileStorage & ifs)
{
    std::string ret;
    // check FileStroage
    if (ifs.isOpened() == false) {
        ret.append("-1\n");
        ret.append("Error: Cannot load camera parameters because of something wrong.\n");
        ret.append("       Cannot open file storage.\n");
        std::cerr << ret;
    }
    // image size
    imgSize = cv::Size(0, 0);
    ifs["imageSize"] >> imgSize;
    if (imgSize.width <= 0 || imgSize.height <= 0) {
        std::cout << "Warning: Read a nonpositive image size from FileStorage.\n";
    }
    // camera matrix
    cmat = cv::Mat();
    ifs["cameraMatrix"] >> cmat;
    if (cmat.cols != 3 || cmat.rows != 3) {
        std::cout << "Warning: Read an invalid camera matrix from FileStorage.\n";
    }
    // distortion vector
    dvec = cv::Mat();
    ifs["distortionVector"] >> dvec;
    if (dvec.cols <= 0 || dvec.rows <= 0) {
        std::cout << "Warning: Read an invalid distortion vector from FileStorage.\n";
    }
    // rvec
    rvec = cv::Mat();
    ifs["rvec"] >> rvec;
    if (rvec.cols <= 0 || rvec.rows <= 0) {
        std::cout << "Warning: Read an invalid rotational vector from FileStorage.\n";
    }
    // tvec
    tvec = cv::Mat();
    ifs["tvec"] >> tvec;
    if (tvec.cols <= 0 || tvec.rows <= 0) {
        std::cout << "Warning: Read an invalid translational vector from FileStroage.\n";
    }
    return std::string("0");
}

/*! \brief Loads camera parameters from a file
 *
 * This function loads camera intrinsic and extrinsic parameters from a file.
 * The tags and formats are:
 * <imageSize> cv::Size
 * <cameraMatrix> cv::Mat(3, 3, CV_64F)
 * <distortionVector> cv::Mat(1, nDistortCoefs, CV_64F)
 * <rvec> cv::Mat(1, 3, CV_64F)
 * <tvec> cv::Mat(1, 3, CV_64F)
 * <R44> cv::Mat(4, 4, CV_64F)
 * <CamPosition> cv::Mat(1, 3, CV_64F)
 * \param imgSize image size cv::Size(width, height)
 * \param cmat camera matrix cv::Mat(3, 3, CV_64F)
 * \param dvec distortion vector cv::Mat(1, nDistortCoefs, CV_64F)
 * \param rvec rotational vector cv::Mat(1, 3, CV_64F)
 * \param tvec translational vector cv::Mat(1, 3, CV_64F)
 * \param fname full file path to load from
 * \return string("0") for success, or warning/error message otherwise.
 * \sa loadCalibrationToFileStorage()
*/
std::string loadCalibrationFromFile(
        cv::Size & imgSize,
        cv::Mat & cmat,
        cv::Mat & dvec,
        cv::Mat & rvec,
        cv::Mat & tvec,
        std::string fname)
{
    std::string ret;
    // check file
    if (fname.length() <= 1) {
        ret.append("-1\n");
        ret.append("Error: Cannot load camera parameters because of something wrong.\n");
        ret.append("       Check file path: " + fname + "\n");
        std::cerr << ret;
        return ret;
    }
    // open file storage to save
    cv::FileStorage ifs(fname, cv::FileStorage::READ);
    if (ifs.isOpened() == false) {
        ret.append("-1\n");
        ret.append("Error: Cannot load camera parameters because of something wrong.\n");
        ret.append("       Cannot open: " + fname + " as a file storage.\n");
        std::cerr << ret;
    }
    ret = loadCalibrationFromFileStorage(imgSize, cmat, dvec, rvec, tvec, ifs);
    if (ret.compare("0") != 0) {
        std::cerr << "Error: loadCalibrationFromFile() receives " << ret
                  << " from loadCalibrationFromFileStorage().\n";
    }
    return std::string("0");
}
