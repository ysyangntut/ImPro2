#ifndef IMPRO_CALIB_H
#define IMPRO_CALIB_H

#include <opencv2/opencv.hpp>

/*! \brief Converts rotational and translational vectors to a 4-by-4 extrinsic matrix.
 *  \param rvec rotational vector, cv::Mat(1, 3, CV_64F)
 *  \param tvec translational vector, cv::Mat(1, 3, CV_64F)
 *  \return 4-by-4 CV_64F matrix of extrinsic matrix
*/
cv::Mat r44_64F(cv::Mat rvec, cv::Mat tvec);

/*! \brief Converts rotational and translational vectors to a 4-by-4 extrinsic matrix.
 *  \param rvec rotational vector, cv::Mat(1, 3, CV_64F)
 *  \param tvec translational vector, cv::Mat(1, 3, CV_64F)
 *  \return 4-by-4 CV_32F matrix of extrinsic matrix
*/
cv::Mat r44_32F(cv::Mat rvec, cv::Mat tvec);

cv::Mat camPos_64F(cv::Mat rvec, cv::Mat tvec);
cv::Mat camPos_32F(cv::Mat rvec, cv::Mat tvec);

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
int checkAndConvertIntrinsic(cv::Mat & cmat, cv::Mat & dvec);

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
int checkAndConvertExtrinsic(cv::Mat & rvec, cv::Mat & tvec);

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
                               cv::Mat & tvec);


/*! \brief Calibrates camera by multiple unregular known points and known
 *         colinear points in a single photo.
 *
 * The function calibrates camera intrinsic and extrinsic parameters
 * according to given known points (with image coordinates and global (world)
 * coordinates) and given colinear points (with only image coordinates).
 * This function first tries to get an initial parameters by calling OpenCV
 * calibrateCamera(), then estimates the global (world) coordinates of colinear
 * points, appends them into aforementioned known points, and adjusts the
 * parameters by calling calibrateCamera() again.
 */
double calibrateCameraSingleImage(
        const cv::Mat & objPoints, //!< global (world) coord. of known points. (1, N, CV_32FC3)
        const cv::Mat & imgPoints, //!< image coord. of known points. (1, N, CV_32FC2)
        const std::vector<cv::Mat> & colinearPoints, //!< image coord. of colinear points. [lineId] (1, nPntThisLine, CV_32FC2)
             cv::Size   imgSize, //!< image size
              cv::Mat & cmat, //!< calculated camera matrix. (3, 3, CV_64F)
              cv::Mat & dvec, //!< calculated distortion vector. (1, nDistCoefs, CV_64F)
              cv::Mat & rvec, //!< calculated rotational vector. (1, 3, CV_64F)
              cv::Mat & tvec, //!< calculated translational vector. (1, 3, CV_64F)
              cv::Mat & stdDeviationsIntrinsics, //!< stdev of intrinsic parameters (1, 4+nDistCoef, CV_64F)
              cv::Mat & stdDeviationsExtrinsics, //!< stdev of extrinsic parameters (1, 6, CV_64F)
              cv::Mat & perViewErrors, //!< (1, 1, CV_64F)
                  int   flags,
     cv::TermCriteria   criteria = cv::TermCriteria(cv::TermCriteria::COUNT +
                                            cv::TermCriteria::EPS,
                                            30, DBL_EPSILON)
        );

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
        cv::FileStorage & ofs);

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
        std::string fname);

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
        cv::FileStorage & ifs);

/*! \brief Loads camera parameters from a file
 *
 * This function loads camera intrinsic and extrinsic parameters from a file.
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
        std::string fname);



int printIntrinsic(
        const cv::Mat & cmat,
        const cv::Mat & dvec,
        const cv::Mat & stdevIntrinsic,
        const cv::Mat & perViewErrors = cv::Mat(0, 0, CV_64F),
        std::ostream & sout = std::cout);

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
        bool showCorners = false,
        bool showUndistorted = false,
        bool printInfo = false,
        int flags = 0,
        cv::TermCriteria criteria =
        cv::TermCriteria(cv::TermCriteria::COUNT +
                         cv::TermCriteria::EPS,
                         30, DBL_EPSILON)
        );




#endif // IMPRO_CALIB_H
