#ifndef USERPOINTCALIBRATIONDIALOG_H
#define USERPOINTCALIBRATIONDIALOG_H

#include <QDialog>
#include <QTableWidgetItem>

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>


namespace Ui {
class UserPointCalibrationDialog;
}

class UserPointCalibrationDialog : public QDialog
{
    Q_OBJECT

public:
    explicit UserPointCalibrationDialog(QWidget *parent = nullptr);
    ~UserPointCalibrationDialog();

    const int nPointsPerColinear = 4;
    cv::Mat  img;
    cv::Mat  imgUndistort;
    std::string imgFilename;
    cv::Mat  cmat, dvec, rvec, tvec;
    cv::Mat  stdDeviationsIntrinsics, stdDeviationsExtrinsics, perViewErrors;
    cv::Mat  uImgPoints;          //!< image coord. of user defined points. (1, nPoints, CV_32FC2)
    cv::Mat  uGlobalPoints;       //!< 3D global coord. of user defined points (1, nPoints, CV_32FC3)
    std::vector<cv::Mat> uColinearImgPoints;  //!< image coord. of user defined lines. [nLines](1, nPointsPerColinear, CV_32FC2)
    cv::Size imgSize;             //!< image size of this camera
    int      flags;               //!< calibration flags

//    QImage opencvMatToQImage(const cv::Mat & img, std::string filename = "");
    QImage opencvMatToQImage(const cv::Mat & img);

private slots:
    void on_pbSelectImg_clicked();

//    void on_pbImgGv_clicked();

    void on_pbImgInteractive_clicked();

    void on_pbImshow1600_clicked();

    // example: input mats as [ (xw1,yw1,zw1), (xw2, yw2, zw2), (xw3, yw3, zw3)], and [(xi1, xi2), (na, na), (xi3, yi3)]
    // output will be
    // [(xw1,yw1,zw1), (xw3, yw3, zw3)],
    // [(xi1, xi2), (xi3, yi3)]
    // , {0, 2} and {0, -1, 1}
    void getValidObjImgPointsAndMappings(cv::Mat objPoints, cv::Mat imgPoints,
                                         cv::Mat & validObjPoints, cv::Mat & validImgPoints,
                                         std::vector<int> & validToAll, std::vector<int> & allToValid);

    void on_edGlobalPoints_textChanged();
    void on_edImgPoints_textChanged();
    void on_edColinearImgPoints_textChanged();

    void on_pbCalibrate_clicked();

    void on_edImgSize_textChanged();

    void on_tbIntrinsic_itemSelectionChanged();
    void on_tbExtrinsic_itemSelectionChanged();
    void on_tbProjection_itemSelectionChanged();

    void on_pbSaveUndistort_clicked();

    void getIntrinsicFromUi();
    void getExtrinsicFromUi();
    void getCalibrationFlagsFromUi();
    void displayIntrinsicToUi();
    void displayExtrinsicToUi();
    void displayProjectionToUi();
    void displayUndistortedImageToUi();

    void displayMessage(QString msg);



    void on_pbSave_clicked();

    void on_pbLoad_clicked();

private:
    Ui::UserPointCalibrationDialog *ui;
};

#endif // USERPOINTCALIBRATIONDIALOG_H
