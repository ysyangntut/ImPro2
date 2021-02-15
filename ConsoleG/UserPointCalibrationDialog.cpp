#include "UserPointCalibrationDialog.h"
#include "ui_UserPointCalibrationDialog.h"

#include <QApplication>
#include <QString>
#include <QFileDialog>
#include <QMessageBox>
#include <QPixmap>
#include <QGraphicsPixmapItem>
#include <QGraphicsView>
#include <QAbstractItemModel>
#include <QClipboard>
#include <QFileInfo>

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <thread>
#include <chrono>
#include <ctime>
#include <sstream>

#include <opencv2/opencv.hpp>

#include "pickAPoint.h"
#include "impro_util.h"
#include "improStrings.h"
#include "improCalib.h"

using std::vector;
using std::cout;
using std::string;
using std::endl;

UserPointCalibrationDialog::UserPointCalibrationDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::UserPointCalibrationDialog)
{
    ui->setupUi(this);
    this->on_edImgSize_textChanged();
    this->on_edGlobalPoints_textChanged();
    this->on_edImgPoints_textChanged();
    this->on_edColinearImgPoints_textChanged();
}

UserPointCalibrationDialog::~UserPointCalibrationDialog()
{
    delete ui;
}

// QImage UserPointCalibrationDialog::opencvMatToQImage(const cv::Mat &img, std::string filename)
QImage UserPointCalibrationDialog::opencvMatToQImage(const cv::Mat & _img)
{
    QImage qimg;
    if (_img.channels() == 3)
        qimg = QImage(_img.data, _img.cols, _img.rows,
                      (int) _img.step, QImage::Format_BGR888);
    else if (img.channels() == 1)
        qimg = QImage(_img.data, _img.cols, _img.rows,
                      (int) _img.step, QImage::Format_Grayscale8);
    else if (_img.channels() == 4)  {
        QMessageBox msgBox;
//        msgBox.setText("Warning: 4-channel image. Assuming ARGB32: " + QString::fromStdString(filename));
        msgBox.setText("Warning: 4-channel image. Assuming ARGB32.");
        msgBox.exec();
        qimg = QImage(_img.data, _img.cols, _img.rows,
                      (int) _img.step, QImage::Format_ARGB32);
    }
    else {
        QMessageBox msgBox;
//        msgBox.setText("Warning: Unknown format: " + QString::fromStdString(filename));
        msgBox.setText("Warning: Unknown format");
        msgBox.exec();
    }
    return qimg;
}

void UserPointCalibrationDialog::on_pbSelectImg_clicked()
{
    // Declare variables
    QString imgQFilename;
    int wLabel = this->ui->lbImg->width();
    int hLabel = this->ui->lbImg->height();


    // Get initial directory
    QString initDir = this->ui->edImgFile->text();
    QFileInfo theFileInfo;
    QDir theDir;
    try {
        if (QDir(initDir).exists() == true) {
            // do nothing, leave initDir unchanged.
        } else
        {
            theFileInfo = QFileInfo(initDir);
            theDir = QDir(theFileInfo.absoluteDir());
            if (theDir.exists() == true)
                initDir = theDir.absolutePath();
            else
                initDir = "./";
        }
    } catch (...) {
        initDir = "./";
    }

    imgQFilename = QFileDialog::getOpenFileName(
                this,
                tr("Select image that contains calibration points"),
                initDir, // "../samples/Demo_SingleImageCalibration/",
                tr("Image Files (*.png *.jpg *.bmp)"));
    if (imgQFilename.length() > 1) {
        this->imgFilename = imgQFilename.toStdString();
        this->ui->edImgFile->setText(imgQFilename);
    }
    else
        return;

    // Read image
    this->img = cv::imread(this->imgFilename);
    if (this->img.cols <= 0 || this->img.rows <= 0)
    {
        QMessageBox msgBox;
        msgBox.setText("Error: Cannot read image from file " + imgQFilename);
        msgBox.exec();
        return;
    }

    // Show image
    QImage qimg;
//    qimg = opencvMatToQImage(this->img, this->imgFilename);
    qimg = opencvMatToQImage(this->img);

    QPixmap qpixmap = QPixmap::fromImage(qimg).scaled(wLabel, hLabel,
                      Qt::KeepAspectRatio, Qt::SmoothTransformation);
    this->ui->lbImg->setPixmap(qpixmap);
}

//void UserPointCalibrationDialog::on_pbImgGv_clicked()
//{
//    // Show image in full resolution with QGraphicsView
//    if (this->img.cols > 0 && this->img.rows > 0)
//    {
//        QGraphicsScene* scene = new QGraphicsScene();
//        QGraphicsView* view = new QGraphicsView(scene);
//        QImage qimg;
//        qimg = opencvMatToQImage(this->img, this->imgFilename);
//        QGraphicsPixmapItem* item = new QGraphicsPixmapItem(QPixmap::fromImage(qimg));
//        scene->addItem(item);
//        view->show();
//    }
//}

void UserPointCalibrationDialog::on_pbImgInteractive_clicked()
{
    cv::Point2f dummyPoint;
    pickAPoint("Interctive view", this->img, dummyPoint);
}

void UserPointCalibrationDialog::on_pbImshow1600_clicked()
{
    imshow_resize("Image Show 1600", this->img, cv::Size(1600, 900));
}



// example: input mats as [ (xw1,yw1,zw1), (xw2, yw2, zw2), (xw3, yw3, zw3)], and [(xi1, xi2), (na, na), (xi3, yi3)]
// output will be
// [(xw1,yw1,zw1), (xw3, yw3, zw3)],
// [(xi1, xi2), (xi3, yi3)]
// , {0, 2} and {0, -1, 1}
void UserPointCalibrationDialog::getValidObjImgPointsAndMappings(cv::Mat objPoints, cv::Mat imgPoints,
                                     cv::Mat & validObjPoints, cv::Mat & validImgPoints,
                                     std::vector<int> & validToAll, std::vector<int> & allToValid)
{
    int nObjPoints = objPoints.cols * objPoints.rows;
    int nImgPoints = imgPoints.cols * imgPoints.rows;
    int nValidPoints = 0;
    allToValid = vector<int>(nObjPoints, -1);
    for (int i = 0; i < std::min(nObjPoints, nImgPoints); i++)
    {
        cv::Point2f ip = imgPoints.at<cv::Point2f>(i);
        cv::Point3f op = objPoints.at<cv::Point3f>(i);
        if (isnan(ip.x) || isnan(ip.y) || isnan(op.x) || isnan(op.y) || isnan(op.z))
        {
            continue;
        }
        else {
            allToValid[i] = nValidPoints;
            nValidPoints++;
        }
    }
    // allocate variables as now we know number of valid points nValidPoints
    validObjPoints = cv::Mat(1, nValidPoints, CV_32FC3);
    validImgPoints = cv::Mat(1, nValidPoints, CV_32FC2);
    validToAll = vector<int>(nValidPoints, -1);
    for (int i = 0; i < std::min(nObjPoints, nImgPoints); i++)
    {
        int iValid = allToValid[i];
        if (iValid >= 0) {
            validToAll[iValid] = i;
            cv::Point3f op = objPoints.at<cv::Point3f>(i);
            cv::Point2f ip = imgPoints.at<cv::Point2f>(i);
            validObjPoints.at<cv::Point3f>(iValid) = op;
            validImgPoints.at<cv::Point2f>(iValid) = ip;
        }
    }
}

void UserPointCalibrationDialog::on_edImgSize_textChanged()
{
    std::stringstream ss;
    std::vector<int> intsInText;

    // init
    this->imgSize = cv::Size(0, 0);
    // get text (entire content to a single string)
    ss << this->ui->edImgSize->toPlainText().toStdString();
    // get data
    for (int i = 0; i < 2; i++)
    {
        int tmp;
        std::string stmp;
        // check EOF of the stream
        if (ss.eof() == true) break;
        // read a string
        ss >> stmp;
        // convert to a float
        try {
            tmp = std::stoi(stmp);
        }  catch (...) {
            tmp = 0;
        }
        // append to vector
        if (tmp >= 1)
            intsInText.push_back(tmp);
    }
    if (intsInText.size() >= 2) {
        this->imgSize.width = intsInText[0];
        this->imgSize.height = intsInText[1];
        this->ui->edImgSize->setStyleSheet("color: black;");
    } else {
        this->ui->edImgSize->setStyleSheet("color: red;");
    }
    return ;
}

void UserPointCalibrationDialog::on_edGlobalPoints_textChanged()
{
    std::vector<float> floatsInText;
    int nFloats, nNanfs, nFloatPoints, nNanfPoints, nPoints;
    // read floats from text
    string text = this->ui->edGlobalPoints->toPlainText().toStdString();
    fromStringToVecFloatAcceptNaIgnoreOthers(text, floatsInText, nFloats, nNanfs);
    // check if number of floats are multiple of three
    this->ui->lbNumGlobalPoints->setStyleSheet("color: black;");
    this->ui->lbNumGlobalPoints->setText("");
    if (nFloats % 3 == 0 && nNanfs % 3 == 0) {
        // if multiple of three, display number of points, and copy to this->uGlobalPoints
        nPoints = (int) (floatsInText.size() / 3);
        nFloatPoints = (int) (nFloats / 3);
        nNanfPoints = (int) (nNanfs / 3);
        if (nPoints != nFloatPoints + nNanfPoints)
        {
            cerr << "Error: Numbers of points, float points, and nanf points are not consistent.\n";
            cerr << "   Quited doing on_edGlobalPoints_textChanged.\n";
            cerr.flush();
            return;
        }
        char buf[1000];
        snprintf(buf, 1000, "%d / %d pts", nFloatPoints, nPoints);
        this->ui->lbNumGlobalPoints->setText(buf);
        this->ui->lbNumGlobalPoints->setStyleSheet("color: black;");
        this->uGlobalPoints = cv::Mat(1, nPoints, CV_32FC3);
        int checkNumFloatPoints = 0;
        for (int i = 0; i < nPoints; i++) {
            float x = floatsInText[i * 3 + 0];
            float y = floatsInText[i * 3 + 1];
            float z = floatsInText[i * 3 + 2];
            if (isnan(x) || isnan(y) || isnan(z)) {
                  this->uGlobalPoints.at<cv::Point3f>(i) =
                        cv::Point3f(nanf(""), nanf(""), nanf(""));
            }
            else {
                this->uGlobalPoints.at<cv::Point3f>(i) = cv::Point3f(x, y, z);
                checkNumFloatPoints++;
            }
        }
        // check numer of calculated float points and added float points are consistent.
        if (checkNumFloatPoints != nFloatPoints) {
            this->ui->lbNumGlobalPoints->setText("");
        }
    }
    if (this->ui->lbNumGlobalPoints->text().length() <= 0)
    {
        // if not multiple of three, display number of floats by red fonts
        char buf[1000];
        snprintf(buf, 1000, "%d / %d floats", nFloats, nNanfs + nFloats);
        this->ui->lbNumGlobalPoints->setText(buf);
        this->ui->lbNumGlobalPoints->setStyleSheet("color: red;");
        this->uGlobalPoints = cv::Mat();
    }
}

void UserPointCalibrationDialog::on_edImgPoints_textChanged()
{
    std::vector<float> floatsInText;
    int nFloats, nNanfs, nFloatPoints, nNanfPoints, nPoints;
    // read floats from text
    string text = this->ui->edImgPoints->toPlainText().toStdString();
    fromStringToVecFloatAcceptNaIgnoreOthers(text, floatsInText, nFloats, nNanfs);
    // check if number of floats are multiple of three
    this->ui->lbNumImgPoints->setStyleSheet("color: black;");
    this->ui->lbNumImgPoints->setText("");
    if (nFloats % 2 == 0 && nNanfs % 2 == 0) {
        // if multiple of two, display number of points, and copy to this->uImgPoints
        nPoints = (int) (floatsInText.size() / 2);
        nFloatPoints = (int) (nFloats / 2);
        nNanfPoints = (int) (nNanfs / 2);
        if (nPoints != nFloatPoints + nNanfPoints)
        {
            cerr << "Error: Numbers of points, float points, and nanf points are not consistent.\n";
            cerr << "   Quited doing on_edImgPoints_textChanged.\n";
            cerr.flush();
            return;
        }
        char buf[1000];
        snprintf(buf, 1000, "%d / %d pts", nFloatPoints, nPoints);
        this->ui->lbNumImgPoints->setText(buf);
        this->ui->lbNumImgPoints->setStyleSheet("color: black;");
        this->uImgPoints = cv::Mat(1, nPoints, CV_32FC2);
        int checkNumFloatPoints = 0;
        for (int i = 0; i < nPoints; i++) {
            float x = floatsInText[i * 2 + 0];
            float y = floatsInText[i * 2 + 1];
            if (isnan(x) || isnan(y)) {
                  this->uImgPoints.at<cv::Point2f>(i) =
                        cv::Point2f(nanf(""), nanf(""));
            }
            else {
                this->uImgPoints.at<cv::Point2f>(i) = cv::Point2f(x, y);
                checkNumFloatPoints++;
            }
        }
        // check numer of calculated float points and added float points are consistent.
        if (checkNumFloatPoints != nFloatPoints) {
            this->ui->lbNumImgPoints->setText("");
        }
    }
    if (this->ui->lbNumImgPoints->text().length() <= 0)
    {
        // if not multiple of three, display number of floats by red fonts
        char buf[1000];
        snprintf(buf, 1000, "%d / %d floats", nFloats, nNanfs + nFloats);
        this->ui->lbNumImgPoints->setText(buf);
        this->ui->lbNumImgPoints->setStyleSheet("color: red;");
        this->uImgPoints = cv::Mat();
    }
}

void UserPointCalibrationDialog::on_edColinearImgPoints_textChanged()
{
    int nLines = 0, nPoints = 0;
    char buf[1000];
    // get string from widget (a long string that may contain multiple "\n")
    string fullText = this->ui->edColinearImgPoints->toPlainText().toStdString();
    bool debug = false;
    // ---------------------------------------------------------
    std::vector<std::vector<float> > vvFloats;
    std::vector<int> nvFloats, nvNanfs;
    int nFloats, nNanfs;
    // If fullText is "1 2 3 4 5 6 # 7 8 9 1 2 3 4 5 ", vvFloats will be
    // vector<vector<float> > {vector<float>{1 2 3 4 5 6}, vector<float>{7 8 9 1 2 3 4 5} }
    // nvFloats will be {6, 8}, nvNanfs will be {0, 0}
    fromStringToVecVecFloatAcceptNaIgnoreOthers(fullText, '#', vvFloats, nvFloats, nvNanfs, nFloats, nNanfs);
    // check validation
    int invalidLine = -1;
    nLines = (int) vvFloats.size();
    for (int i = 0; i < nLines; i++) {
        // if number of floats is even (so that it forms Point2f) without #N/A
        if (nvFloats[i] % 2 != 0 || nvNanfs[i] != 0 || nvFloats[i] / 2 < 3) {
            invalidLine = i;
            break;
        }
    }
    if (invalidLine >= 0) {
        snprintf(buf, 1000, "ERR@ line %d", invalidLine + 1);
        this->ui->lbNumColinearLinesAndPoints->setText(buf);
        this->ui->lbNumColinearLinesAndPoints->setStyleSheet("color: red;");
        return;
    }
    if (invalidLine < 0) {
        this->uColinearImgPoints.clear();
        this->uColinearImgPoints.resize(nLines);
        // If vvFloats is
        // vector<vector<float> > {vector<float>{1 2 3 4 5 6}, vector<float>{7 8 9 1 2 3 4 5} }
        // this->uColinearImgPoints will be
        // vector<cv::Mat>{ cv::Point2f(1, 2)
        for (int i = 0; i < nLines; i++) {
            cv::Mat tmp(1, nvFloats[i] / 2, CV_32FC2, vvFloats[i].data());
            this->uColinearImgPoints[i] = tmp.clone();
            if (debug) {
                std::cout << "Colinear " << i + 1 << " is:\n" << tmp << "\n";
                std::cout.flush();
            }
        }
        // display
        snprintf(buf, 1000, "%d / %d", nLines, nFloats / 2);
        this->ui->lbNumColinearLinesAndPoints->setText(buf);
        this->ui->lbNumColinearLinesAndPoints->setStyleSheet("color: black;");
    }
    return;
    // ---------------------------------------------------------
    // convert entire string into strings
    std::stringstream streamFullText(fullText);
    std::vector<std::string> strings;
    if (fullText.length() > 0) {
        std::string thisLine;
//        while(std::getline(streamFullText, thisLine, '\n'))
        while(std::getline(streamFullText, thisLine, '#'))
        {
            // if thisLine has nothing (only spaces, TABs, ...) then ignore thisLine
            std::stringstream streamThisLine(thisLine);
            std::string testIfEmpty;
            streamThisLine >> testIfEmpty;
            if (testIfEmpty.length() >= 1) {
                // OK. There is something to read. Add it to strings.
                strings.push_back(thisLine);
            }
            else
            {
                 // nothing to read from thisLine. Ignore it.
            }
        }
    }
//    // for debug
//    for (int i = 0; i < (int) strings.size(); i++) {
//        printf("%d %s\n", i, strings[i].c_str());
//        fflush(stdout); cout.flush();
//    }

    // From vector<string> to vector<Point2f>
    // each string in strings (vector<string>) represents a line of text, which
    // represents multiple colinear points of a straight line.
    //
    nLines = (int) strings.size();
    this->uColinearImgPoints.clear();
    this->uColinearImgPoints.resize(nLines);
    bool somethingWrong = false;
    for (int i = 0; i < nLines; i++)
    {
        std::vector<cv::Point2f> colinearPointsThisLine;
        std::stringstream streamThisLine(strings[i]);
        // read points
        while (true) {
            std::string str_x, str_y;
            float x, y;
            streamThisLine >> str_x >> str_y;
            if (str_x.length() <= 0 && str_y.length() <= 0) break;
            try {
                x = y = nanf("");
                x = std::stof(str_x);
                y = std::stof(str_y);
                if (isnan(x) == false && isnan(y) == false)
                    colinearPointsThisLine.push_back(cv::Point2f(x, y));
                else
                    somethingWrong = true;
            } catch (...) {
                somethingWrong = true;
            }
            // if something wrong this line, display red line number
            if (somethingWrong == true) {
                snprintf(buf, 1000, "ERR@ line %d", i + 1);
                this->ui->lbNumColinearLinesAndPoints->setText(buf);
                this->ui->lbNumColinearLinesAndPoints->setStyleSheet("color: red;");
                break;
            }
        } // end of while loop of each point in this line
        // if something wrong, clear this->uColinearImgPoints and about to exit this function
        if (somethingWrong == true) {
            this->uColinearImgPoints.clear();
            break;
        }
        // check there are at least three points of this straight line
        int nPointsThisLine = (int) colinearPointsThisLine.size();
        if (nPointsThisLine < 3) {
            somethingWrong = true;
            snprintf(buf, 1000, "ERR@ line %d", i + 1);
            this->ui->lbNumColinearLinesAndPoints->setText(buf);
            this->ui->lbNumColinearLinesAndPoints->setStyleSheet("color: red;");
            break;
        }
        // if this line is okay, add colinearPointsThisLine into this->uColinearImgPoints
        cv::Mat colinearPointsThisLineMat(1, nPointsThisLine, CV_32FC2);
        for (int j = 0; j < nPointsThisLine; j++)
            colinearPointsThisLineMat.at<cv::Point2f>(j) = colinearPointsThisLine[j];
        this->uColinearImgPoints[i] = colinearPointsThisLineMat;
        nPoints += nPointsThisLine;
        // debug
        cout << "Line " << i + 1 << "\n";
        cout << this->uColinearImgPoints[i] << endl;
        cout.flush();
    } // end of while loop of each line

    // display information
    if (somethingWrong == false) {
        snprintf(buf, 1000, "%d / %d", nLines, nPoints);
        this->ui->lbNumColinearLinesAndPoints->setText(buf);
        this->ui->lbNumColinearLinesAndPoints->setStyleSheet("color: black;");
    }

}

void UserPointCalibrationDialog::getIntrinsicFromUi()
{
    // get intrinsic from ui widgets to this->cmat and this->dvec
    QTableWidgetItem *pCell;
    double tmp;
    bool parseOK;
    this->cmat = cv::Mat::eye(3, 3, CV_64F);
    this->dvec = cv::Mat::zeros(1, 12, CV_64F);
    pCell = this->ui->tbIntrinsic->item(0, 0);
    if (pCell != NULL) {
        tmp = pCell->text().toDouble(&parseOK);
        if (parseOK == false) tmp = 0.0;
        cmat.at<double>(0, 0) = tmp;
    }
    pCell = this->ui->tbIntrinsic->item(1, 0);
    if (pCell != NULL) {
        tmp = pCell->text().toDouble(&parseOK);
        if (parseOK == false) tmp = 0.0;
        cmat.at<double>(1, 1) = tmp;
    }
    pCell = this->ui->tbIntrinsic->item(2, 0);
    if (pCell != NULL) {
        tmp = pCell->text().toDouble(&parseOK);
        if (parseOK == false) tmp = 0.0;
        cmat.at<double>(0, 2) = tmp;
    }
    pCell = this->ui->tbIntrinsic->item(3, 0);
    if (pCell != NULL) {
        tmp = pCell->text().toDouble(&parseOK);
        if (parseOK == false) tmp = 0.0;
        cmat.at<double>(1, 2) = tmp;
    }
    for (int i = 0; i < 12; i++) {
        pCell = this->ui->tbIntrinsic->item(i + 4, 0);
        if (pCell != NULL) {
            tmp = pCell->text().toDouble(&parseOK);
            if (parseOK == false) tmp = 0.0;
            dvec.at<double>(i) = tmp;
        }
    }
    cout << "Intrinsic parameters: \n" << cmat << "\n" << dvec << endl;
}

void UserPointCalibrationDialog::getExtrinsicFromUi()
{
    // get intrinsic from ui widgets to this->cmat and this->dvec
    QTableWidgetItem *pCell;
    double tmp;
    bool parseOK;
    this->rvec = cv::Mat::zeros(3, 1, CV_64F);
    this->tvec = cv::Mat::zeros(3, 1, CV_64F);
    pCell = this->ui->tbExtrinsic->item(0, 0);
    for (int i = 0; i < 3; i++) {
        pCell = this->ui->tbExtrinsic->item(i, 0);
        if (pCell != NULL) {
            tmp = pCell->text().toDouble(&parseOK);
            if (parseOK == false) tmp = 0.0;
            rvec.at<double>(i) = tmp;
        }
    }
    for (int i = 3; i < 6; i++) {
        pCell = this->ui->tbExtrinsic->item(i, 0);
        if (pCell != NULL) {
            tmp = pCell->text().toDouble(&parseOK);
            if (parseOK == false) tmp = 0.0;
            tvec.at<double>(i - 3) = tmp;
        }
    }
    cout << "Extrinsic parameters: \n" << rvec << "\n" << tvec << endl;

}

void UserPointCalibrationDialog::getCalibrationFlagsFromUi()
{
    // get flags
    this->flags = this->ui->cb01->isChecked() * cv::CALIB_USE_INTRINSIC_GUESS +
            this->ui->cb02->isChecked() * cv::CALIB_FIX_ASPECT_RATIO    +
            this->ui->cb03->isChecked() * cv::CALIB_FIX_PRINCIPAL_POINT +
            this->ui->cb04->isChecked() * cv::CALIB_ZERO_TANGENT_DIST   +
            this->ui->cb05->isChecked() * cv::CALIB_FIX_FOCAL_LENGTH    +
            this->ui->cb06->isChecked() * cv::CALIB_FIX_K1              +
            this->ui->cb07->isChecked() * cv::CALIB_FIX_K2              +
            this->ui->cb08->isChecked() * cv::CALIB_FIX_K3              +
            this->ui->cb09->isChecked() * cv::CALIB_FIX_K4              +
            this->ui->cb10->isChecked() * cv::CALIB_FIX_K5              +
            this->ui->cb11->isChecked() * cv::CALIB_FIX_K6              +
            this->ui->cb12->isChecked() * cv::CALIB_RATIONAL_MODEL      +
            this->ui->cb13->isChecked() * cv::CALIB_THIN_PRISM_MODEL    +
            this->ui->cb14->isChecked() * cv::CALIB_FIX_S1_S2_S3_S4     +
            this->ui->cb15->isChecked() * cv::CALIB_TILTED_MODEL        +
            this->ui->cb16->isChecked() * cv::CALIB_FIX_TAUX_TAUY       +
            this->ui->cb17->isChecked() * cv::CALIB_USE_QR              +
            this->ui->cb18->isChecked() * cv::CALIB_FIX_TANGENT_DIST    +
            this->ui->cb19->isChecked() * cv::CALIB_FIX_INTRINSIC       +
            this->ui->cb20->isChecked() * cv::CALIB_SAME_FOCAL_LENGTH   +
            this->ui->cb21->isChecked() * cv::CALIB_ZERO_DISPARITY      +
            this->ui->cb22->isChecked() * cv::CALIB_USE_LU              +
            this->ui->cb23->isChecked() * cv::CALIB_USE_EXTRINSIC_GUESS;
}

void UserPointCalibrationDialog::displayIntrinsicToUi()
{
    // declaration
    char buf[1000];
    // display intrinsic parameters

    // allocate cells if necessary
    QTableWidgetItem *pCell;
    for (int i = 0; i < 18; i++) {
        for (int j = 0; j < 2; j++) {
            pCell = this->ui->tbIntrinsic->item(i, j);
            if (!pCell) {
                pCell = new QTableWidgetItem;
                this->ui->tbIntrinsic->setItem(i, j, pCell);
            }
            this->ui->tbIntrinsic->item(i, j)->setText("");
        }
    }
    // display cmat to table widget
    snprintf(buf, 1000, "%.16f", cmat.at<double>(0, 0));
    this->ui->tbIntrinsic->item(0, 0)->setText(buf);
    snprintf(buf, 1000, "%.16f", cmat.at<double>(1, 1));
    this->ui->tbIntrinsic->item(1, 0)->setText(buf);
    snprintf(buf, 1000, "%.16f", cmat.at<double>(0, 2));
    this->ui->tbIntrinsic->item(2, 0)->setText(buf);
    snprintf(buf, 1000, "%.16f", cmat.at<double>(1, 2));
    this->ui->tbIntrinsic->item(3, 0)->setText(buf);
    // display distortion coefficients
    for (int i = 0; i < dvec.cols * dvec.rows; i++) {
        snprintf(buf, 1000, "%24.16e", dvec.at<double>(i));
        this->ui->tbIntrinsic->item(i + 4, 0)->setText(buf);
    }
    // display standard deviation
    for (int i = 0; i < this->stdDeviationsIntrinsics.cols *
         this->stdDeviationsIntrinsics.rows; i++) {
        snprintf(buf, 1000, "%24.16e", this->stdDeviationsIntrinsics.at<double>(i));
        this->ui->tbIntrinsic->item(i, 1)->setText(buf);
    }
}

void UserPointCalibrationDialog::displayExtrinsicToUi()
{
    // display extrinsic parameters
    char buf[1000];

    // display rvec, tvec
    QTableWidgetItem *pCell;
    for (int i = 0; i < 9; i++)
    {
        pCell = this->ui->tbExtrinsic->item(i, 0);
        if (!pCell) {
            pCell = new QTableWidgetItem;
            this->ui->tbExtrinsic->setItem(i, 0, pCell);
        }
        this->ui->tbExtrinsic->item(i, 0)->setText("");
    }
    // display this->rvec
//    cout << "rvec: " << rvec << "\n"; cout.flush(); cout.flush(); cout.flush();
//    cout << "rvec size: " << rvec.rows << " " << rvec.cols << "\n";
    cout.flush(); cout.flush(); cout.flush();
    if (rvec.cols * rvec.rows == 3) {
        cout << "rvec (3): " << rvec << "\n"; cout.flush(); cout.flush(); cout.flush();
        snprintf(buf, 1000, "%.16f", rvec.at<double>(0));
        cout << "rvec [0]: " << buf << "\n";
        this->ui->tbExtrinsic->item(0, 0)->setText(buf);
        snprintf(buf, 1000, "%.16f", rvec.at<double>(1));
        this->ui->tbExtrinsic->item(1, 0)->setText(buf);
        snprintf(buf, 1000, "%.16f", rvec.at<double>(2));
        this->ui->tbExtrinsic->item(2, 0)->setText(buf);
        cout << "rvec: " << rvec << "\n"; cout.flush(); cout.flush(); cout.flush();
    }
    // display this->tvec
//    cout << "rvec: " << rvec << "\n"; cout.flush(); cout.flush(); cout.flush();
    if (tvec.cols * tvec.rows == 3) {
        snprintf(buf, 1000, "%.16f", tvec.at<double>(0));
        this->ui->tbExtrinsic->item(3, 0)->setText(buf);
        snprintf(buf, 1000, "%.16f", tvec.at<double>(1));
        this->ui->tbExtrinsic->item(4, 0)->setText(buf);
        snprintf(buf, 1000, "%.16f", tvec.at<double>(2));
        this->ui->tbExtrinsic->item(5, 0)->setText(buf);
    }
    // convert rvec and tvec to campos
    if (rvec.cols * rvec.rows == 3 && tvec.cols * tvec.rows == 3) {
        cv::Mat R4 = cv::Mat::eye(4, 4, CV_64F), R4inv;
        cv::Mat R3 = R4(cv::Rect(0, 0, 3, 3));
        R4.at<double>(0, 3) = tvec.at<double>(0);
        R4.at<double>(1, 3) = tvec.at<double>(1);
        R4.at<double>(2, 3) = tvec.at<double>(2);
        cv::Rodrigues(rvec, R3);
        R4inv = R4.inv();
        // display camera position
        snprintf(buf, 1000, "%.16f", R4inv.at<double>(0, 3));
        this->ui->tbExtrinsic->item(6, 0)->setText(buf);
        snprintf(buf, 1000, "%.16f", R4inv.at<double>(1, 3));
        this->ui->tbExtrinsic->item(7, 0)->setText(buf);
        snprintf(buf, 1000, "%.16f", R4inv.at<double>(2, 3));
        this->ui->tbExtrinsic->item(8, 0)->setText(buf);
    }
}

void UserPointCalibrationDialog::displayProjectionToUi()
{
    char buf[1000];
    int nGlobalPoints = this->uGlobalPoints.cols * this->uGlobalPoints.rows;
    int nImgPoints = this->uImgPoints.cols * this->uImgPoints.rows;
    vector<int> validToAll, allToValid;
    int nValidPoints = 0;
    cv::Mat validGlobalPoints, validPrjPoints;
    // get valid points and index mappings between valid points and all points
    this->getValidObjImgPointsAndMappings(this->uGlobalPoints, this->uImgPoints,
                                    validGlobalPoints, validPrjPoints,
                                    validToAll, allToValid);
    nValidPoints = (int) validToAll.size();
//    // create Mats which have only valid global images and image points
//    validGlobalPoints = cv::Mat::zeros(1, nValidPoints, CV_32FC3);
//    validPrjPoints = cv::Mat::zeros(1, nValidPoints, CV_32FC2);
//    We commented this because this assumed all points are valid (no nanf points)
//    // check point data consistancy
//    int nImgPoints = this->uImgPoints.cols * this->uImgPoints.rows;
//    cv::Mat uPrjPoints; // (this->uImgPoints);
//    // check data
//    if (nImgPoints != nGlobalPoints || nImgPoints == 0)
//    {
//        // return
//        snprintf(buf, 1000, "Warning: Numbers of image points (%d) "
//        "and global points (%d) are not consistent.\n", nImgPoints, nGlobalPoints);
//        this->displayMessage(buf);
//        this->ui->tbProjection->clearContents();
//        return;
//    }
    // get parameters from ui
    try {
        this->getIntrinsicFromUi();
        this->getExtrinsicFromUi();
    } catch (...)
    {
        snprintf(buf, 1000, "Warning: Failed to get camera "
                            "parameters from ui widgets.\n");
        this->displayMessage(buf);
        this->ui->tbProjection->clearContents();
        return;
    }
    // point projection (only the valid points)
    try {
        cv::projectPoints(validGlobalPoints,
                          this->rvec, this->tvec, this->cmat, this->dvec,
                          validPrjPoints);
    } catch (...) {
        snprintf(buf, 1000, "Warning: Failed to do point projection "
                 "by data from ui widgets.\n");
        this->displayMessage(buf);
        return;
    }
    // display data to widgets
    QTableWidgetItem *pCell;
    this->ui->tbProjection->setRowCount(nImgPoints);
    this->ui->tbProjection->setColumnCount(4);
    for (int i = 0; i < std::min(nGlobalPoints, nImgPoints); i++) {
        // allocate cells if necessary
        for (int j = 0; j < 4; j++) {
            pCell = this->ui->tbProjection->item(i, j);
            if (!pCell) {
                pCell = new QTableWidgetItem;
                this->ui->tbProjection->setItem(i, j, pCell);
            }
        }
        // if invalid, fill "NA".
        int validIdx = allToValid[i];
        if (validIdx <= -1) {
            this->ui->tbProjection->item(i, 0)->setText("");
            this->ui->tbProjection->item(i, 1)->setText("");
            this->ui->tbProjection->item(i, 2)->setText("");
            this->ui->tbProjection->item(i, 3)->setText("");
            continue;
        }
        cv::Point2f ip = this->uImgPoints.at<cv::Point2f>(i);
        cv::Point2f pp = validPrjPoints.at<cv::Point2f>(validIdx);
        cv::Point3f op = this->uGlobalPoints.at<cv::Point3f>(i);

        // display projected xp
        snprintf(buf, 1000, "%24.16e", pp.x);
        this->ui->tbProjection->item(i, 0)->setText(buf);
        // display projected yp
        snprintf(buf, 1000, "%24.16e", pp.y);
        this->ui->tbProjection->item(i, 1)->setText(buf);
        // display projected xp - xi
        snprintf(buf, 1000, "%.4f", pp.x - ip.x);
        this->ui->tbProjection->item(i, 2)->setText(buf);
        // display projected yp - yi
        snprintf(buf, 1000, "%.4f", pp.y - ip.y);
        this->ui->tbProjection->item(i, 3)->setText(buf);
    }
    return;
}

void UserPointCalibrationDialog::displayUndistortedImageToUi()
{
    bool debug = false;
    // check if image exists
    if (this->img.cols <= 0 || this->img.rows <= 0) return;

    // get parameters from ui widgets to this->cmat and this->dvec
    this->getIntrinsicFromUi();

    // undistort image from this->img to this->imgUndistort
    if (this->imgUndistort.cols <= 0 || this->imgUndistort.rows <= 0)
        cv::undistort(this->img, this->imgUndistort, this->cmat, this->dvec, this->cmat);

    if (debug) {
        cout << "cmat:\n" << this->cmat << endl;
        cout << "dvec:\n" << this->dvec << endl;
        cout.flush();
        imshow_resize("try undistorted", this->imgUndistort, cv::Size(1600, 800));
    }

    // display undistorted image in ui
    int wLabel = this->ui->lbImgUndistorted->width();
    int hLabel = this->ui->lbImgUndistorted->height();
    QImage qimg;
    qimg = opencvMatToQImage(this->imgUndistort);
    QPixmap qpixmap = QPixmap::fromImage(qimg).scaled(wLabel, hLabel,
                      Qt::KeepAspectRatio, Qt::SmoothTransformation);
    this->ui->lbImgUndistorted->setPixmap(qpixmap);

}

void UserPointCalibrationDialog::displayMessage(QString msg)
{
    QMessageBox mbox;
    mbox.setText(msg);
    mbox.exec();
    return;
}


void UserPointCalibrationDialog::on_pbCalibrate_clicked()
{
    // declarations
    cv::Mat colinearPoints;
    cv::Size imgSize;
    vector<int> validToAll, allToValid;
    int nValidPoints = 0;
    cv::Mat validGlobalPoints, validImgPoints;
    // data check
    if (this->uGlobalPoints.cols * this->uGlobalPoints.rows <= 0 ||
        this->uImgPoints.cols * this->uImgPoints.rows <= 0 ||
        this->uGlobalPoints.cols * this->uGlobalPoints.rows !=
        this->uImgPoints.cols * this->uImgPoints.rows)
    {
        QMessageBox mbox;
        mbox.setText("Error: Invalid # of 3D points and 2D points.");
        mbox.exec();
        return;
    }
    // get index mappings between valid points and all points
    getValidObjImgPointsAndMappings(this->uGlobalPoints, this->uImgPoints,
                                    validGlobalPoints, validImgPoints,
                                    validToAll, allToValid);
    nValidPoints = (int) validToAll.size();

    // get image size
    imgSize = this->imgSize;
    if (imgSize.area() <= 0) {
        QMessageBox mbox;
        mbox.setText("Error: Invalid image size.");
        mbox.exec();
        return;
    }

    // get flags from ui
    this->getCalibrationFlagsFromUi();

    // get initial guess of cmat and dvec
    if (this->ui->cb01->isChecked()) {
        this->getIntrinsicFromUi();
    }

    // calibraion
//    vector<cv::Mat> rvecs(1, cv::Mat::zeros(3, 1, CV_64F)), tvecs(1, cv::Mat::zeros(3, 1, CV_64F));
    double rms =
    calibrateCameraSingleImage(validGlobalPoints,
                               validImgPoints,
                               this->uColinearImgPoints,
                               imgSize,
                               this->cmat,
                               this->dvec,
                               this->rvec,
                               this->tvec,
                               this->stdDeviationsIntrinsics,
                               this->stdDeviationsExtrinsics,
                               this->perViewErrors,
                               this->flags);
    printf("Calibration rms: %24.16e\n", rms); fflush(stdout);
    // from some version of calibrateCamera(), rvec and tvec are
    // in 3-channel format, i.e., cv::Mat(?, ?, CV_64FC3)
    // so we convert them to 1-channel Mats, and each of them has only 1 row

    rvec = rvec.reshape(1);
    tvec = tvec.reshape(1);

    // update information to ui
    this->displayIntrinsicToUi();
    this->displayExtrinsicToUi();
    this->displayProjectionToUi();

    // clear undistort image as parameters have been changed.
    this->imgUndistort = cv::Mat(0, 0, CV_8UC3);

    // display undistorted image in a QLabel
    this->displayUndistortedImageToUi();

    return;
}



void UserPointCalibrationDialog::on_tbIntrinsic_itemSelectionChanged()
{
    // When items in this table widget are selected,
    // selected items are also copied to clipboard.
    // PS: We reproduced this Qt manipulation skill from Internet.

    QAbstractItemModel * model = this->ui->tbIntrinsic->model();
    QItemSelectionModel * selection = this->ui->tbIntrinsic->selectionModel();
    QModelIndexList indexes = selection->selectedIndexes();
    QString selected_text;
    QModelIndex previous = indexes.last();
//    indexes.removeFirst();
    foreach(const QModelIndex &current, indexes)
    {
        QVariant data = model->data(current);
        QString text = data.toString();
        selected_text.append(text);
        if (current.row() != previous.row())
            selected_text.append('\n');
        else
            selected_text.append('\t');
        previous = current;
    }
    static_cast<QApplication *>(QApplication::instance())
            ->clipboard()->setText(selected_text);
//   cout << "------------------\n";
//   cout << selected_text.toStdString() << "\n";
//   cout << "------------------\n";
//   cout.flush();
    //   cout.flush();
}

void UserPointCalibrationDialog::on_tbExtrinsic_itemSelectionChanged()
{
    // When items in this table widget are selected,
    // selected items are also copied to clipboard.
    // PS: We reproduced this Qt manipulation skill from Internet.

    QAbstractItemModel * model = this->ui->tbExtrinsic->model();
    QItemSelectionModel * selection = this->ui->tbExtrinsic->selectionModel();
    QModelIndexList indexes = selection->selectedIndexes();
    QString selected_text;
    QModelIndex previous = indexes.last();
    foreach(const QModelIndex &current, indexes)
    {
        QVariant data = model->data(current);
        QString text = data.toString();
        selected_text.append(text);
        if (current.row() != previous.row())
            selected_text.append('\n');
        else
            selected_text.append('\t');
        previous = current;
    }
    static_cast<QApplication *>(QApplication::instance())
            ->clipboard()->setText(selected_text);
}

void UserPointCalibrationDialog::on_tbProjection_itemSelectionChanged()
{
    // When items in this table widget are selected,
    // selected items are also copied to clipboard.
    // PS: We reproduced this Qt manipulation skill from Internet.

    QAbstractItemModel * model = this->ui->tbProjection->model();
    QItemSelectionModel * selection = this->ui->tbProjection->selectionModel();
    QModelIndexList indexes = selection->selectedIndexes();
    QString selected_text;
    QModelIndex previous = indexes.last();
    foreach(const QModelIndex &current, indexes)
    {
        QVariant data = model->data(current);
        QString text = data.toString();
        selected_text.append(text);
        if (current.row() != previous.row())
            selected_text.append('\n');
        else
            selected_text.append('\t');
        previous = current;
    }
    static_cast<QApplication *>(QApplication::instance())
            ->clipboard()->setText(selected_text);
}


void UserPointCalibrationDialog::on_pbSaveUndistort_clicked()
{
    // check if image exists
    if (this->img.cols <= 0 || this->img.rows <= 0) return;

    // get parameters from ui widgets to this->cmat and this->dvec
    this->getIntrinsicFromUi();

    // undistort image from this->img to this->imgUndistort
    if (this->imgUndistort.cols <= 0 || this->imgUndistort.rows <= 0)
        cv::undistort(this->img, this->imgUndistort, this->cmat, this->dvec, this->cmat);

    // show image
    imshow_resize("Undistorted image", this->imgUndistort, cv::Size(1600, 900));

    // Get initial directory
    QString initDir = this->ui->edImgFile->text();
    QFileInfo theFileInfo;
    QDir theDir;
    try {
        if (QDir(initDir).exists() == true) {
            // do nothing, leave initDir unchanged.
        } else
        {
            theFileInfo = QFileInfo(initDir);
            theDir = QDir(theFileInfo.absoluteDir());
            if (theDir.exists() == true)
                initDir = theDir.absolutePath();
            else
                initDir = "./";
        }
    } catch (...) {
        initDir = "./";
    }

    // ask if want to save undistort file
    QString imgUndistortFile;
    imgUndistortFile = QFileDialog::getSaveFileName
            (this, tr("Save Undistorted Image to File"), initDir, tr("Images (*.png *.bmp *.jpg)"));

    // if file name is valid, save the undistorted image to a file
    if (imgUndistortFile.length() >= 1) {
        cv::imwrite(imgUndistortFile.toStdString(), this->imgUndistort);
    }
}



void UserPointCalibrationDialog::on_pbSave_clicked()
{
    //
    // Get initial directory
    QString initDir = this->ui->edImgFile->text();
    QFileInfo theFileInfo;
    QDir theDir;
    try {
        if (QDir(initDir).exists() == true) {
            // do nothing, leave initDir unchanged.
        } else
        {
            theFileInfo = QFileInfo(initDir);
            theDir = QDir(theFileInfo.absoluteDir());
            if (theDir.exists() == true)
                initDir = theDir.absolutePath();
            else
                initDir = "./";
        }
    } catch (...) {
        initDir = "./";
    }

    QString Qfilepath = QFileDialog::getSaveFileName(
                this,
                tr("Save camera parameters to XML file"),
                initDir,
                tr("XML Files (*.xml)"));
    if (Qfilepath.length() <= 1) return;
    std::string retStringSaveCalib =
            saveCalibrationToFile(
                this->imgSize,
                this->cmat,
                this->dvec,
                this->rvec,
                this->tvec,
                Qfilepath.toStdString());
    return;
}

void UserPointCalibrationDialog::on_pbLoad_clicked()
{
    // Get initial directory
    QString initDir = this->ui->edImgFile->text();
    QFileInfo theFileInfo;
    QDir theDir;
    try {
        if (QDir(initDir).exists() == true) {
            // do nothing, leave initDir unchanged.
        } else
        {
            theFileInfo = QFileInfo(initDir);
            theDir = QDir(theFileInfo.absoluteDir());
            if (theDir.exists() == true)
                initDir = theDir.absolutePath();
            else
                initDir = "./";
        }
    } catch (...) {
        initDir = "./";
    }

    // get file from user
    QString Qfilepath = QFileDialog::getOpenFileName(
                this,
                tr("Load camera parameters from XML file"),
                initDir,
                tr("XML Files (*.xml)"));
    if (Qfilepath.length() <= 1) return;
    // read data from file
    std::string retStringLoadCalib =
            loadCalibrationFromFile(
                this->imgSize,
                this->cmat,
                this->dvec,
                this->rvec,
                this->tvec,
                Qfilepath.toStdString());
    // update information to ui
    this->displayIntrinsicToUi();
    this->displayExtrinsicToUi();
    this->displayProjectionToUi();

    // clear undistort image as parameters have been changed.
    this->imgUndistort = cv::Mat(0, 0, CV_8UC3);

}

void UserPointCalibrationDialog::on_pbDrawMesh_clicked()
{



    // get parameters from Ui

}
