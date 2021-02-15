/********************************************************************************
** Form generated from reading UI file 'UserPointCalibrationDialog.ui'
**
** Created by: Qt User Interface Compiler version 5.15.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_USERPOINTCALIBRATIONDIALOG_H
#define UI_USERPOINTCALIBRATIONDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDialog>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QTableWidget>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QVBoxLayout>

QT_BEGIN_NAMESPACE

class Ui_UserPointCalibrationDialog
{
public:
    QPushButton *pbSelectImg;
    QLineEdit *edImgFile;
    QLabel *lbImg;
    QPushButton *pbImgInteractive;
    QPushButton *pbImshow1600;
    QLabel *lb1;
    QTextEdit *edGlobalPoints;
    QLabel *lbNumGlobalPoints;
    QTextEdit *edImgPoints;
    QLabel *lbNumImgPoints;
    QLabel *lb2;
    QLabel *lbNumColinearLinesAndPoints;
    QLabel *lb3;
    QTextEdit *edColinearImgPoints;
    QLabel *lb1_2;
    QTextEdit *edImgSize;
    QPushButton *pbCalibrate;
    QGroupBox *groupBox;
    QVBoxLayout *vboxLayout;
    QCheckBox *cb01;
    QCheckBox *cb02;
    QCheckBox *cb03;
    QCheckBox *cb04;
    QCheckBox *cb05;
    QCheckBox *cb06;
    QCheckBox *cb07;
    QCheckBox *cb08;
    QCheckBox *cb09;
    QCheckBox *cb10;
    QCheckBox *cb11;
    QCheckBox *cb12;
    QCheckBox *cb13;
    QCheckBox *cb14;
    QCheckBox *cb15;
    QCheckBox *cb16;
    QCheckBox *cb17;
    QCheckBox *cb18;
    QCheckBox *cb19;
    QCheckBox *cb20;
    QCheckBox *cb21;
    QCheckBox *cb22;
    QCheckBox *cb23;
    QTableWidget *tbIntrinsic;
    QTableWidget *tbExtrinsic;
    QTableWidget *tbProjection;
    QPushButton *pbUndistort;
    QPushButton *pbLoad;
    QPushButton *pbSave;
    QLabel *lbImgUndistorted;
    QLabel *label;
    QPushButton *pbImshow1600Undistorted;
    QLabel *label_2;
    QPushButton *pbImgInteractiveUndistorted;

    void setupUi(QDialog *UserPointCalibrationDialog)
    {
        if (UserPointCalibrationDialog->objectName().isEmpty())
            UserPointCalibrationDialog->setObjectName(QString::fromUtf8("UserPointCalibrationDialog"));
        UserPointCalibrationDialog->resize(1880, 960);
        UserPointCalibrationDialog->setStyleSheet(QString::fromUtf8(""));
        pbSelectImg = new QPushButton(UserPointCalibrationDialog);
        pbSelectImg->setObjectName(QString::fromUtf8("pbSelectImg"));
        pbSelectImg->setGeometry(QRect(8, 20, 101, 28));
        QFont font;
        font.setFamily(QString::fromUtf8("Consolas"));
        font.setPointSize(8);
        pbSelectImg->setFont(font);
        edImgFile = new QLineEdit(UserPointCalibrationDialog);
        edImgFile->setObjectName(QString::fromUtf8("edImgFile"));
        edImgFile->setGeometry(QRect(110, 20, 471, 31));
        edImgFile->setFont(font);
        lbImg = new QLabel(UserPointCalibrationDialog);
        lbImg->setObjectName(QString::fromUtf8("lbImg"));
        lbImg->setGeometry(QRect(590, 18, 601, 341));
        lbImg->setMouseTracking(false);
        lbImg->setFrameShape(QFrame::Box);
        pbImgInteractive = new QPushButton(UserPointCalibrationDialog);
        pbImgInteractive->setObjectName(QString::fromUtf8("pbImgInteractive"));
        pbImgInteractive->setGeometry(QRect(720, 360, 131, 31));
        pbImgInteractive->setFont(font);
        pbImshow1600 = new QPushButton(UserPointCalibrationDialog);
        pbImshow1600->setObjectName(QString::fromUtf8("pbImshow1600"));
        pbImshow1600->setGeometry(QRect(850, 360, 111, 31));
        pbImshow1600->setFont(font);
        lb1 = new QLabel(UserPointCalibrationDialog);
        lb1->setObjectName(QString::fromUtf8("lb1"));
        lb1->setGeometry(QRect(26, 85, 91, 31));
        QFont font1;
        font1.setFamily(QString::fromUtf8("Consolas"));
        lb1->setFont(font1);
        lb1->setAutoFillBackground(true);
        lb1->setWordWrap(true);
        edGlobalPoints = new QTextEdit(UserPointCalibrationDialog);
        edGlobalPoints->setObjectName(QString::fromUtf8("edGlobalPoints"));
        edGlobalPoints->setGeometry(QRect(110, 83, 411, 71));
        edGlobalPoints->setFont(font);
        edGlobalPoints->setLineWrapMode(QTextEdit::NoWrap);
        lbNumGlobalPoints = new QLabel(UserPointCalibrationDialog);
        lbNumGlobalPoints->setObjectName(QString::fromUtf8("lbNumGlobalPoints"));
        lbNumGlobalPoints->setGeometry(QRect(8, 120, 101, 31));
        lbNumGlobalPoints->setFont(font1);
        lbNumGlobalPoints->setStyleSheet(QString::fromUtf8(""));
        lbNumGlobalPoints->setFrameShape(QFrame::Box);
        lbNumGlobalPoints->setWordWrap(true);
        edImgPoints = new QTextEdit(UserPointCalibrationDialog);
        edImgPoints->setObjectName(QString::fromUtf8("edImgPoints"));
        edImgPoints->setGeometry(QRect(110, 169, 411, 71));
        edImgPoints->setFont(font);
        edImgPoints->setLineWrapMode(QTextEdit::NoWrap);
        lbNumImgPoints = new QLabel(UserPointCalibrationDialog);
        lbNumImgPoints->setObjectName(QString::fromUtf8("lbNumImgPoints"));
        lbNumImgPoints->setGeometry(QRect(8, 209, 101, 31));
        lbNumImgPoints->setFont(font1);
        lbNumImgPoints->setStyleSheet(QString::fromUtf8(""));
        lbNumImgPoints->setFrameShape(QFrame::Box);
        lbNumImgPoints->setWordWrap(true);
        lb2 = new QLabel(UserPointCalibrationDialog);
        lb2->setObjectName(QString::fromUtf8("lb2"));
        lb2->setGeometry(QRect(10, 167, 91, 41));
        lb2->setFont(font1);
        lb2->setAutoFillBackground(true);
        lb2->setWordWrap(true);
        lbNumColinearLinesAndPoints = new QLabel(UserPointCalibrationDialog);
        lbNumColinearLinesAndPoints->setObjectName(QString::fromUtf8("lbNumColinearLinesAndPoints"));
        lbNumColinearLinesAndPoints->setGeometry(QRect(8, 300, 101, 31));
        lbNumColinearLinesAndPoints->setFont(font1);
        lbNumColinearLinesAndPoints->setStyleSheet(QString::fromUtf8(""));
        lbNumColinearLinesAndPoints->setFrameShape(QFrame::Box);
        lbNumColinearLinesAndPoints->setWordWrap(true);
        lb3 = new QLabel(UserPointCalibrationDialog);
        lb3->setObjectName(QString::fromUtf8("lb3"));
        lb3->setGeometry(QRect(11, 258, 101, 41));
        lb3->setFont(font1);
        lb3->setAutoFillBackground(true);
        lb3->setWordWrap(true);
        edColinearImgPoints = new QTextEdit(UserPointCalibrationDialog);
        edColinearImgPoints->setObjectName(QString::fromUtf8("edColinearImgPoints"));
        edColinearImgPoints->setGeometry(QRect(110, 260, 411, 71));
        edColinearImgPoints->setFont(font);
        edColinearImgPoints->setLineWrapMode(QTextEdit::NoWrap);
        lb1_2 = new QLabel(UserPointCalibrationDialog);
        lb1_2->setObjectName(QString::fromUtf8("lb1_2"));
        lb1_2->setGeometry(QRect(18, 56, 91, 21));
        lb1_2->setFont(font1);
        lb1_2->setAutoFillBackground(true);
        lb1_2->setWordWrap(true);
        edImgSize = new QTextEdit(UserPointCalibrationDialog);
        edImgSize->setObjectName(QString::fromUtf8("edImgSize"));
        edImgSize->setGeometry(QRect(110, 53, 121, 28));
        edImgSize->setFont(font1);
        pbCalibrate = new QPushButton(UserPointCalibrationDialog);
        pbCalibrate->setObjectName(QString::fromUtf8("pbCalibrate"));
        pbCalibrate->setGeometry(QRect(370, 400, 301, 71));
        pbCalibrate->setFont(font);
        groupBox = new QGroupBox(UserPointCalibrationDialog);
        groupBox->setObjectName(QString::fromUtf8("groupBox"));
        groupBox->setGeometry(QRect(10, 360, 321, 531));
        QFont font2;
        font2.setFamily(QString::fromUtf8("Consolas"));
        font2.setBold(true);
        font2.setWeight(75);
        groupBox->setFont(font2);
        vboxLayout = new QVBoxLayout(groupBox);
        vboxLayout->setSpacing(6);
        vboxLayout->setObjectName(QString::fromUtf8("vboxLayout"));
        vboxLayout->setSizeConstraint(QLayout::SetMinimumSize);
        vboxLayout->setContentsMargins(-1, 3, -1, 3);
        cb01 = new QCheckBox(groupBox);
        cb01->setObjectName(QString::fromUtf8("cb01"));
        QFont font3;
        font3.setFamily(QString::fromUtf8("Consolas"));
        font3.setPointSize(8);
        font3.setBold(true);
        font3.setWeight(75);
        cb01->setFont(font3);
        cb01->setChecked(false);

        vboxLayout->addWidget(cb01);

        cb02 = new QCheckBox(groupBox);
        cb02->setObjectName(QString::fromUtf8("cb02"));
        cb02->setFont(font3);

        vboxLayout->addWidget(cb02);

        cb03 = new QCheckBox(groupBox);
        cb03->setObjectName(QString::fromUtf8("cb03"));
        cb03->setFont(font3);

        vboxLayout->addWidget(cb03);

        cb04 = new QCheckBox(groupBox);
        cb04->setObjectName(QString::fromUtf8("cb04"));
        cb04->setFont(font3);

        vboxLayout->addWidget(cb04);

        cb05 = new QCheckBox(groupBox);
        cb05->setObjectName(QString::fromUtf8("cb05"));
        cb05->setFont(font3);

        vboxLayout->addWidget(cb05);

        cb06 = new QCheckBox(groupBox);
        cb06->setObjectName(QString::fromUtf8("cb06"));
        cb06->setFont(font3);

        vboxLayout->addWidget(cb06);

        cb07 = new QCheckBox(groupBox);
        cb07->setObjectName(QString::fromUtf8("cb07"));
        cb07->setFont(font3);

        vboxLayout->addWidget(cb07);

        cb08 = new QCheckBox(groupBox);
        cb08->setObjectName(QString::fromUtf8("cb08"));
        cb08->setFont(font3);

        vboxLayout->addWidget(cb08);

        cb09 = new QCheckBox(groupBox);
        cb09->setObjectName(QString::fromUtf8("cb09"));
        cb09->setFont(font3);

        vboxLayout->addWidget(cb09);

        cb10 = new QCheckBox(groupBox);
        cb10->setObjectName(QString::fromUtf8("cb10"));
        cb10->setFont(font3);

        vboxLayout->addWidget(cb10);

        cb11 = new QCheckBox(groupBox);
        cb11->setObjectName(QString::fromUtf8("cb11"));
        cb11->setFont(font3);

        vboxLayout->addWidget(cb11);

        cb12 = new QCheckBox(groupBox);
        cb12->setObjectName(QString::fromUtf8("cb12"));
        cb12->setFont(font3);

        vboxLayout->addWidget(cb12);

        cb13 = new QCheckBox(groupBox);
        cb13->setObjectName(QString::fromUtf8("cb13"));
        cb13->setFont(font3);

        vboxLayout->addWidget(cb13);

        cb14 = new QCheckBox(groupBox);
        cb14->setObjectName(QString::fromUtf8("cb14"));
        cb14->setFont(font3);

        vboxLayout->addWidget(cb14);

        cb15 = new QCheckBox(groupBox);
        cb15->setObjectName(QString::fromUtf8("cb15"));
        cb15->setFont(font3);

        vboxLayout->addWidget(cb15);

        cb16 = new QCheckBox(groupBox);
        cb16->setObjectName(QString::fromUtf8("cb16"));
        cb16->setFont(font3);

        vboxLayout->addWidget(cb16);

        cb17 = new QCheckBox(groupBox);
        cb17->setObjectName(QString::fromUtf8("cb17"));
        cb17->setFont(font3);

        vboxLayout->addWidget(cb17);

        cb18 = new QCheckBox(groupBox);
        cb18->setObjectName(QString::fromUtf8("cb18"));
        cb18->setFont(font3);

        vboxLayout->addWidget(cb18);

        cb19 = new QCheckBox(groupBox);
        cb19->setObjectName(QString::fromUtf8("cb19"));
        cb19->setFont(font3);

        vboxLayout->addWidget(cb19);

        cb20 = new QCheckBox(groupBox);
        cb20->setObjectName(QString::fromUtf8("cb20"));
        cb20->setFont(font3);

        vboxLayout->addWidget(cb20);

        cb21 = new QCheckBox(groupBox);
        cb21->setObjectName(QString::fromUtf8("cb21"));
        cb21->setFont(font3);

        vboxLayout->addWidget(cb21);

        cb22 = new QCheckBox(groupBox);
        cb22->setObjectName(QString::fromUtf8("cb22"));
        cb22->setFont(font3);

        vboxLayout->addWidget(cb22);

        cb23 = new QCheckBox(groupBox);
        cb23->setObjectName(QString::fromUtf8("cb23"));
        cb23->setFont(font3);

        vboxLayout->addWidget(cb23);

        tbIntrinsic = new QTableWidget(UserPointCalibrationDialog);
        if (tbIntrinsic->columnCount() < 2)
            tbIntrinsic->setColumnCount(2);
        QTableWidgetItem *__qtablewidgetitem = new QTableWidgetItem();
        __qtablewidgetitem->setFont(font);
        tbIntrinsic->setHorizontalHeaderItem(0, __qtablewidgetitem);
        QTableWidgetItem *__qtablewidgetitem1 = new QTableWidgetItem();
        tbIntrinsic->setHorizontalHeaderItem(1, __qtablewidgetitem1);
        if (tbIntrinsic->rowCount() < 18)
            tbIntrinsic->setRowCount(18);
        QTableWidgetItem *__qtablewidgetitem2 = new QTableWidgetItem();
        tbIntrinsic->setVerticalHeaderItem(0, __qtablewidgetitem2);
        QTableWidgetItem *__qtablewidgetitem3 = new QTableWidgetItem();
        tbIntrinsic->setVerticalHeaderItem(1, __qtablewidgetitem3);
        QTableWidgetItem *__qtablewidgetitem4 = new QTableWidgetItem();
        tbIntrinsic->setVerticalHeaderItem(2, __qtablewidgetitem4);
        QTableWidgetItem *__qtablewidgetitem5 = new QTableWidgetItem();
        tbIntrinsic->setVerticalHeaderItem(3, __qtablewidgetitem5);
        QTableWidgetItem *__qtablewidgetitem6 = new QTableWidgetItem();
        tbIntrinsic->setVerticalHeaderItem(4, __qtablewidgetitem6);
        QTableWidgetItem *__qtablewidgetitem7 = new QTableWidgetItem();
        tbIntrinsic->setVerticalHeaderItem(5, __qtablewidgetitem7);
        QTableWidgetItem *__qtablewidgetitem8 = new QTableWidgetItem();
        tbIntrinsic->setVerticalHeaderItem(6, __qtablewidgetitem8);
        QTableWidgetItem *__qtablewidgetitem9 = new QTableWidgetItem();
        tbIntrinsic->setVerticalHeaderItem(7, __qtablewidgetitem9);
        QTableWidgetItem *__qtablewidgetitem10 = new QTableWidgetItem();
        tbIntrinsic->setVerticalHeaderItem(8, __qtablewidgetitem10);
        QTableWidgetItem *__qtablewidgetitem11 = new QTableWidgetItem();
        tbIntrinsic->setVerticalHeaderItem(9, __qtablewidgetitem11);
        QTableWidgetItem *__qtablewidgetitem12 = new QTableWidgetItem();
        tbIntrinsic->setVerticalHeaderItem(10, __qtablewidgetitem12);
        QTableWidgetItem *__qtablewidgetitem13 = new QTableWidgetItem();
        tbIntrinsic->setVerticalHeaderItem(11, __qtablewidgetitem13);
        QTableWidgetItem *__qtablewidgetitem14 = new QTableWidgetItem();
        tbIntrinsic->setVerticalHeaderItem(12, __qtablewidgetitem14);
        QTableWidgetItem *__qtablewidgetitem15 = new QTableWidgetItem();
        tbIntrinsic->setVerticalHeaderItem(13, __qtablewidgetitem15);
        QTableWidgetItem *__qtablewidgetitem16 = new QTableWidgetItem();
        tbIntrinsic->setVerticalHeaderItem(14, __qtablewidgetitem16);
        QTableWidgetItem *__qtablewidgetitem17 = new QTableWidgetItem();
        tbIntrinsic->setVerticalHeaderItem(15, __qtablewidgetitem17);
        QTableWidgetItem *__qtablewidgetitem18 = new QTableWidgetItem();
        tbIntrinsic->setVerticalHeaderItem(16, __qtablewidgetitem18);
        QTableWidgetItem *__qtablewidgetitem19 = new QTableWidgetItem();
        tbIntrinsic->setVerticalHeaderItem(17, __qtablewidgetitem19);
        tbIntrinsic->setObjectName(QString::fromUtf8("tbIntrinsic"));
        tbIntrinsic->setGeometry(QRect(370, 480, 271, 192));
        tbIntrinsic->setFont(font);
        tbIntrinsic->setRowCount(18);
        tbIntrinsic->setColumnCount(2);
        tbIntrinsic->horizontalHeader()->setMinimumSectionSize(24);
        tbIntrinsic->horizontalHeader()->setDefaultSectionSize(200);
        tbIntrinsic->verticalHeader()->setMinimumSectionSize(22);
        tbIntrinsic->verticalHeader()->setDefaultSectionSize(25);
        tbExtrinsic = new QTableWidget(UserPointCalibrationDialog);
        if (tbExtrinsic->columnCount() < 2)
            tbExtrinsic->setColumnCount(2);
        QTableWidgetItem *__qtablewidgetitem20 = new QTableWidgetItem();
        __qtablewidgetitem20->setFont(font);
        tbExtrinsic->setHorizontalHeaderItem(0, __qtablewidgetitem20);
        QTableWidgetItem *__qtablewidgetitem21 = new QTableWidgetItem();
        tbExtrinsic->setHorizontalHeaderItem(1, __qtablewidgetitem21);
        if (tbExtrinsic->rowCount() < 9)
            tbExtrinsic->setRowCount(9);
        QTableWidgetItem *__qtablewidgetitem22 = new QTableWidgetItem();
        tbExtrinsic->setVerticalHeaderItem(0, __qtablewidgetitem22);
        QTableWidgetItem *__qtablewidgetitem23 = new QTableWidgetItem();
        tbExtrinsic->setVerticalHeaderItem(1, __qtablewidgetitem23);
        QTableWidgetItem *__qtablewidgetitem24 = new QTableWidgetItem();
        tbExtrinsic->setVerticalHeaderItem(2, __qtablewidgetitem24);
        QTableWidgetItem *__qtablewidgetitem25 = new QTableWidgetItem();
        tbExtrinsic->setVerticalHeaderItem(3, __qtablewidgetitem25);
        QTableWidgetItem *__qtablewidgetitem26 = new QTableWidgetItem();
        tbExtrinsic->setVerticalHeaderItem(4, __qtablewidgetitem26);
        QTableWidgetItem *__qtablewidgetitem27 = new QTableWidgetItem();
        tbExtrinsic->setVerticalHeaderItem(5, __qtablewidgetitem27);
        QTableWidgetItem *__qtablewidgetitem28 = new QTableWidgetItem();
        tbExtrinsic->setVerticalHeaderItem(6, __qtablewidgetitem28);
        QTableWidgetItem *__qtablewidgetitem29 = new QTableWidgetItem();
        tbExtrinsic->setVerticalHeaderItem(7, __qtablewidgetitem29);
        QTableWidgetItem *__qtablewidgetitem30 = new QTableWidgetItem();
        tbExtrinsic->setVerticalHeaderItem(8, __qtablewidgetitem30);
        tbExtrinsic->setObjectName(QString::fromUtf8("tbExtrinsic"));
        tbExtrinsic->setGeometry(QRect(370, 680, 271, 201));
        tbExtrinsic->setFont(font);
        tbExtrinsic->setRowCount(9);
        tbExtrinsic->setColumnCount(2);
        tbExtrinsic->horizontalHeader()->setMinimumSectionSize(24);
        tbExtrinsic->horizontalHeader()->setDefaultSectionSize(200);
        tbExtrinsic->verticalHeader()->setMinimumSectionSize(22);
        tbExtrinsic->verticalHeader()->setDefaultSectionSize(25);
        tbProjection = new QTableWidget(UserPointCalibrationDialog);
        if (tbProjection->columnCount() < 4)
            tbProjection->setColumnCount(4);
        QTableWidgetItem *__qtablewidgetitem31 = new QTableWidgetItem();
        __qtablewidgetitem31->setFont(font);
        tbProjection->setHorizontalHeaderItem(0, __qtablewidgetitem31);
        QTableWidgetItem *__qtablewidgetitem32 = new QTableWidgetItem();
        tbProjection->setHorizontalHeaderItem(1, __qtablewidgetitem32);
        QTableWidgetItem *__qtablewidgetitem33 = new QTableWidgetItem();
        tbProjection->setHorizontalHeaderItem(2, __qtablewidgetitem33);
        QTableWidgetItem *__qtablewidgetitem34 = new QTableWidgetItem();
        tbProjection->setHorizontalHeaderItem(3, __qtablewidgetitem34);
        if (tbProjection->rowCount() < 3)
            tbProjection->setRowCount(3);
        QTableWidgetItem *__qtablewidgetitem35 = new QTableWidgetItem();
        tbProjection->setVerticalHeaderItem(0, __qtablewidgetitem35);
        QTableWidgetItem *__qtablewidgetitem36 = new QTableWidgetItem();
        tbProjection->setVerticalHeaderItem(1, __qtablewidgetitem36);
        QTableWidgetItem *__qtablewidgetitem37 = new QTableWidgetItem();
        tbProjection->setItem(0, 0, __qtablewidgetitem37);
        tbProjection->setObjectName(QString::fromUtf8("tbProjection"));
        tbProjection->setGeometry(QRect(650, 480, 541, 401));
        tbProjection->setFont(font);
        tbProjection->setRowCount(3);
        tbProjection->setColumnCount(4);
        tbProjection->horizontalHeader()->setVisible(false);
        tbProjection->horizontalHeader()->setMinimumSectionSize(24);
        tbProjection->horizontalHeader()->setDefaultSectionSize(200);
        tbProjection->verticalHeader()->setMinimumSectionSize(22);
        tbProjection->verticalHeader()->setDefaultSectionSize(22);
        pbUndistort = new QPushButton(UserPointCalibrationDialog);
        pbUndistort->setObjectName(QString::fromUtf8("pbUndistort"));
        pbUndistort->setGeometry(QRect(680, 400, 171, 71));
        pbUndistort->setFont(font);
        pbLoad = new QPushButton(UserPointCalibrationDialog);
        pbLoad->setObjectName(QString::fromUtf8("pbLoad"));
        pbLoad->setGeometry(QRect(860, 400, 131, 71));
        pbLoad->setFont(font);
        pbSave = new QPushButton(UserPointCalibrationDialog);
        pbSave->setObjectName(QString::fromUtf8("pbSave"));
        pbSave->setGeometry(QRect(1000, 400, 191, 71));
        pbSave->setFont(font);
        lbImgUndistorted = new QLabel(UserPointCalibrationDialog);
        lbImgUndistorted->setObjectName(QString::fromUtf8("lbImgUndistorted"));
        lbImgUndistorted->setGeometry(QRect(1200, 18, 601, 341));
        lbImgUndistorted->setMouseTracking(false);
        lbImgUndistorted->setFrameShape(QFrame::Box);
        label = new QLabel(UserPointCalibrationDialog);
        label->setObjectName(QString::fromUtf8("label"));
        label->setGeometry(QRect(600, 363, 111, 20));
        label->setFont(font3);
        pbImshow1600Undistorted = new QPushButton(UserPointCalibrationDialog);
        pbImshow1600Undistorted->setObjectName(QString::fromUtf8("pbImshow1600Undistorted"));
        pbImshow1600Undistorted->setGeometry(QRect(1493, 362, 111, 31));
        pbImshow1600Undistorted->setFont(font);
        label_2 = new QLabel(UserPointCalibrationDialog);
        label_2->setObjectName(QString::fromUtf8("label_2"));
        label_2->setGeometry(QRect(1220, 364, 111, 20));
        label_2->setFont(font3);
        pbImgInteractiveUndistorted = new QPushButton(UserPointCalibrationDialog);
        pbImgInteractiveUndistorted->setObjectName(QString::fromUtf8("pbImgInteractiveUndistorted"));
        pbImgInteractiveUndistorted->setGeometry(QRect(1363, 362, 131, 31));
        pbImgInteractiveUndistorted->setFont(font);
        QWidget::setTabOrder(cb09, edImgFile);
        QWidget::setTabOrder(edImgFile, edImgSize);
        QWidget::setTabOrder(edImgSize, edGlobalPoints);
        QWidget::setTabOrder(edGlobalPoints, edImgPoints);
        QWidget::setTabOrder(edImgPoints, edColinearImgPoints);
        QWidget::setTabOrder(edColinearImgPoints, cb01);
        QWidget::setTabOrder(cb01, cb02);
        QWidget::setTabOrder(cb02, cb03);
        QWidget::setTabOrder(cb03, cb07);
        QWidget::setTabOrder(cb07, cb08);
        QWidget::setTabOrder(cb08, cb06);
        QWidget::setTabOrder(cb06, cb04);
        QWidget::setTabOrder(cb04, cb10);
        QWidget::setTabOrder(cb10, cb11);
        QWidget::setTabOrder(cb11, cb12);
        QWidget::setTabOrder(cb12, pbImgInteractive);
        QWidget::setTabOrder(pbImgInteractive, pbImshow1600);
        QWidget::setTabOrder(pbImshow1600, cb13);
        QWidget::setTabOrder(cb13, cb14);
        QWidget::setTabOrder(cb14, cb15);
        QWidget::setTabOrder(cb15, pbCalibrate);
        QWidget::setTabOrder(pbCalibrate, cb17);
        QWidget::setTabOrder(cb17, cb18);
        QWidget::setTabOrder(cb18, cb19);
        QWidget::setTabOrder(cb19, cb16);
        QWidget::setTabOrder(cb16, cb23);
        QWidget::setTabOrder(cb23, cb22);
        QWidget::setTabOrder(cb22, cb20);
        QWidget::setTabOrder(cb20, cb21);
        QWidget::setTabOrder(cb21, pbSelectImg);
        QWidget::setTabOrder(pbSelectImg, cb05);

        retranslateUi(UserPointCalibrationDialog);

        QMetaObject::connectSlotsByName(UserPointCalibrationDialog);
    } // setupUi

    void retranslateUi(QDialog *UserPointCalibrationDialog)
    {
        UserPointCalibrationDialog->setWindowTitle(QCoreApplication::translate("UserPointCalibrationDialog", "Dialog", nullptr));
        pbSelectImg->setText(QCoreApplication::translate("UserPointCalibrationDialog", "Select &Image", nullptr));
        edImgFile->setText(QCoreApplication::translate("UserPointCalibrationDialog", "../samples/Demo_SingleImageCalibration/", nullptr));
        lbImg->setText(QString());
        pbImgInteractive->setText(QCoreApplication::translate("UserPointCalibrationDialog", "Interactive view", nullptr));
        pbImshow1600->setText(QCoreApplication::translate("UserPointCalibrationDialog", "Img Show 1600", nullptr));
        lb1->setText(QCoreApplication::translate("UserPointCalibrationDialog", "Global 3D points", nullptr));
        edGlobalPoints->setHtml(QCoreApplication::translate("UserPointCalibrationDialog", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Consolas'; font-size:8pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">10.00	0.00	35.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">10.00	0.00	235.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">10.00	0.00	435.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">10.00	0.00	635.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; t"
                        "ext-indent:0px;\">10.00	0.00	835.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">10.00	0.00	1035.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">10.00	0.00	1235.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">135.00	215.00	35.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">135.00	215.00	235.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">135.00	215.00	435.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">135.00	215.00	635.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-"
                        "block-indent:0; text-indent:0px;\">135.00	215.00	835.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">135.00	215.00	1035.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">135.00	215.00	1235.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">2150.00	0.00	35.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">2150.00	0.00	235.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">2150.00	0.00	435.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">2150.00	0.00	635.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; "
                        "margin-right:0px; -qt-block-indent:0; text-indent:0px;\">2150.00	0.00	835.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">2150.00	0.00	1035.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">2150.00	0.00	1235.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">2025.00	10.00	35.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">2025.00	10.00	235.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">2025.00	10.00	435.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">2025.00	10.00	635.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom"
                        ":0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">2025.00	10.00	835.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">2025.00	10.00	1035.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">2025.00	10.00	1235.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">180.00	45.00	1650.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">480.00	45.00	1650.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">780.00	45.00	1650.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">1080.00	45.00	1650.00</p>\n"
"<p style=\" m"
                        "argin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">1380.00	45.00	1650.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">1680.00	45.00	1650.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">1980.00	45.00	1650.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">2150.00	225.00	35.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">2150.00	225.00	235.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">2150.00	225.00	435.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">2150.00	225."
                        "00	635.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">2150.00	225.00	835.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">2150.00	225.00	1035.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">2150.00	225.00	1235.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">10.00	225.00	35.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">10.00	225.00	235.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">10.00	225.00	435.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text"
                        "-indent:0px;\">10.00	225.00	635.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">10.00	225.00	835.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">10.00	225.00	1035.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">10.00	225.00	1235.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">1980.00	180.00	1650.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">1680.00	180.00	1650.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">1380.00	180.00	1650.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:"
                        "0px; -qt-block-indent:0; text-indent:0px;\">1080.00	180.00	1650.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">780.00	180.00	1650.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">480.00	180.00	1650.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">180.00	180.00	1650.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">2160.00	10.00	35.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">2160.00	10.00	235</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">2160.00	10.00	435</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-"
                        "left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">2160.00	10.00	635</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">2160.00	10.00	835</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">2160.00	10.00	1035</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">2160.00	10.00	1235</p></body></html>", nullptr));
#if QT_CONFIG(tooltip)
        lbNumGlobalPoints->setToolTip(QCoreApplication::translate("UserPointCalibrationDialog", "Numbers of valid points / all points (or, if not multiple of 3, showing numbers of valid floats and all floats.", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(statustip)
        lbNumGlobalPoints->setStatusTip(QString());
#endif // QT_CONFIG(statustip)
#if QT_CONFIG(whatsthis)
        lbNumGlobalPoints->setWhatsThis(QCoreApplication::translate("UserPointCalibrationDialog", "Numbers of valid points / all points (or, if not multiple of 3, showing numbers of valid floats and all floats.", nullptr));
#endif // QT_CONFIG(whatsthis)
        lbNumGlobalPoints->setText(QCoreApplication::translate("UserPointCalibrationDialog", "<html><head/><body><p>1/1</p></body></html>", nullptr));
        edImgPoints->setHtml(QCoreApplication::translate("UserPointCalibrationDialog", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Consolas'; font-size:8pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">1122.95	3683.65</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">1117.57	3265.93</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">1110.63	2844.90</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">1108.55	2425.79</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-ind"
                        "ent:0px;\">1109.48	2006.52</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">1114.63	1588.94</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">1124.49	1179.89</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">NA	NA</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">NA	NA</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">NA	NA</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">NA	NA</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">NA	NA</p>\n"
"<p style=\" margin-top:0p"
                        "x; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">NA	NA</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">NA	NA</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">4954.31	3387.71</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">4953.32	3052.60</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">4950.69	2717.74</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">4946.15	2384.09</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">4940.28	2052.17</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-l"
                        "eft:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">4929.45	1721.40</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">4915.83	1393.63</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">4768.86	3398.98</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">4767.17	3060.90</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">4766.23	2720.23</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">4761.27	2380.53</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">4753.88	2043.40</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; m"
                        "argin-right:0px; -qt-block-indent:0; text-indent:0px;\">4743.52	1707.67</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">4728.94	1372.87</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">NA	NA</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">2107.09	456.40</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">2667.53	507.12</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">3200.77	560.19</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">3700.16	614.40</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-blo"
                        "ck-indent:0; text-indent:0px;\">4166.48	668.61</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">4603.31	714.65</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">NA	NA</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">NA	NA</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">NA	NA</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">NA	NA</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">NA	NA</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">NA	NA</p>\n"
"<p style=\" marg"
                        "in-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">NA	NA</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">NA	NA</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">NA	NA</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">NA	NA</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">NA	NA</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">NA	NA</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">NA	NA</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-inde"
                        "nt:0; text-indent:0px;\">NA	NA</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">NA	NA</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">NA	NA</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">NA	NA</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">NA	NA</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">NA	NA</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">NA	NA</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">NA	NA</p>\n"
"<p style=\" margin-top:0px; margin-bottom"
                        ":0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">NA	NA</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">NA	NA</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">NA	NA</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">NA	NA</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">NA	NA</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">NA	NA</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">NA	NA</p></body></html>", nullptr));
#if QT_CONFIG(tooltip)
        lbNumImgPoints->setToolTip(QCoreApplication::translate("UserPointCalibrationDialog", "Numbers of valid points / all points (or, if not multiple of 2, showing numbers of valid floats and all floats.", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(whatsthis)
        lbNumImgPoints->setWhatsThis(QCoreApplication::translate("UserPointCalibrationDialog", "Numbers of valid points / all points (or, if not multiple of 2, showing numbers of valid floats and all floats.", nullptr));
#endif // QT_CONFIG(whatsthis)
        lbNumImgPoints->setText(QCoreApplication::translate("UserPointCalibrationDialog", "1/1", nullptr));
        lb2->setText(QCoreApplication::translate("UserPointCalibrationDialog", "Image points", nullptr));
#if QT_CONFIG(tooltip)
        lbNumColinearLinesAndPoints->setToolTip(QCoreApplication::translate("UserPointCalibrationDialog", "Numbers of points / lines. Straight lines are splitted by #. Points between two #s are colinear along a straight line.  Each point is represented by two floats. If number of floats are not multiple of 2, or a line has less than 3 points, it shows the first invalid line in red. ", nullptr));
#endif // QT_CONFIG(tooltip)
#if QT_CONFIG(whatsthis)
        lbNumColinearLinesAndPoints->setWhatsThis(QCoreApplication::translate("UserPointCalibrationDialog", "Numbers of points / lines. Straight lines are splitted by #. Points between two #s are colinear along a straight line.  Each point is represented by two floats. If number of floats are not multiple of 2, or a line has less than 3 points, it shows the first invalid line in red. ", nullptr));
#endif // QT_CONFIG(whatsthis)
        lbNumColinearLinesAndPoints->setText(QString());
        lb3->setText(QCoreApplication::translate("UserPointCalibrationDialog", "Colinear image points", nullptr));
        edColinearImgPoints->setHtml(QCoreApplication::translate("UserPointCalibrationDialog", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Consolas'; font-size:8pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">1700.00	998.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">2485.00	1045.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">3502.00	1111.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">4364.00	1178.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-inde"
                        "nt:0px;\">4484.00	1184.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">#	</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">1386.00	1183.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">1375.00	1835.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">1376.00	2664.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">1390.00	3503.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">#	</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">4780.00	3271.00</p>\n"
"<p"
                        " style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">4778.00	2728.00</p>\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">4759.00	1793.00</p></body></html>", nullptr));
        lb1_2->setText(QCoreApplication::translate("UserPointCalibrationDialog", "Image size", nullptr));
        edImgSize->setHtml(QCoreApplication::translate("UserPointCalibrationDialog", "<!DOCTYPE HTML PUBLIC \"-//W3C//DTD HTML 4.0//EN\" \"http://www.w3.org/TR/REC-html40/strict.dtd\">\n"
"<html><head><meta name=\"qrichtext\" content=\"1\" /><style type=\"text/css\">\n"
"p, li { white-space: pre-wrap; }\n"
"</style></head><body style=\" font-family:'Consolas'; font-size:9pt; font-weight:400; font-style:normal;\">\n"
"<p style=\" margin-top:0px; margin-bottom:0px; margin-left:0px; margin-right:0px; -qt-block-indent:0; text-indent:0px;\">6000 4000</p></body></html>", nullptr));
        pbCalibrate->setText(QCoreApplication::translate("UserPointCalibrationDialog", "&Calibrate", nullptr));
        groupBox->setTitle(QCoreApplication::translate("UserPointCalibrationDialog", "Calibration Flags", nullptr));
        cb01->setText(QCoreApplication::translate("UserPointCalibrationDialog", "CALIB_USE_INTRINSIC_GUESS", nullptr));
        cb02->setText(QCoreApplication::translate("UserPointCalibrationDialog", "CALIB_FIX_ASPECT_RATIO", nullptr));
        cb03->setText(QCoreApplication::translate("UserPointCalibrationDialog", "CALIB_FIX_PRINCIPAL_POINT ", nullptr));
        cb04->setText(QCoreApplication::translate("UserPointCalibrationDialog", "CALIB_ZERO_TANGENT_DIST ", nullptr));
        cb05->setText(QCoreApplication::translate("UserPointCalibrationDialog", "CALIB_FIX_FOCAL_LENGTH ", nullptr));
        cb06->setText(QCoreApplication::translate("UserPointCalibrationDialog", "CALIB_FIX_K1 ", nullptr));
        cb07->setText(QCoreApplication::translate("UserPointCalibrationDialog", "CALIB_FIX_K2", nullptr));
        cb08->setText(QCoreApplication::translate("UserPointCalibrationDialog", "CALIB_FIX_K3", nullptr));
        cb09->setText(QCoreApplication::translate("UserPointCalibrationDialog", "CALIB_FIX_K4", nullptr));
        cb10->setText(QCoreApplication::translate("UserPointCalibrationDialog", "CALIB_FIX_K5", nullptr));
        cb11->setText(QCoreApplication::translate("UserPointCalibrationDialog", "CALIB_FIX_K6", nullptr));
        cb12->setText(QCoreApplication::translate("UserPointCalibrationDialog", "CALIB_RATIONAL_MODEL", nullptr));
        cb13->setText(QCoreApplication::translate("UserPointCalibrationDialog", "CALIB_THIN_PRISM_MODEL ", nullptr));
        cb14->setText(QCoreApplication::translate("UserPointCalibrationDialog", "CALIB_FIX_S1_S2_S3_S4", nullptr));
        cb15->setText(QCoreApplication::translate("UserPointCalibrationDialog", "CALIB_TILTED_MODEL", nullptr));
        cb16->setText(QCoreApplication::translate("UserPointCalibrationDialog", "CALIB_FIX_TAUX_TAUY ", nullptr));
        cb17->setText(QCoreApplication::translate("UserPointCalibrationDialog", "CALIB_USE_QR ", nullptr));
        cb18->setText(QCoreApplication::translate("UserPointCalibrationDialog", "CALIB_FIX_TANGENT_DIST", nullptr));
        cb19->setText(QCoreApplication::translate("UserPointCalibrationDialog", "CALIB_FIX_INTRINSIC ", nullptr));
        cb20->setText(QCoreApplication::translate("UserPointCalibrationDialog", "CALIB_SAME_FOCAL_LENGTH ", nullptr));
        cb21->setText(QCoreApplication::translate("UserPointCalibrationDialog", "CALIB_ZERO_DISPARITY", nullptr));
        cb22->setText(QCoreApplication::translate("UserPointCalibrationDialog", "CALIB_USE_LU", nullptr));
        cb23->setText(QCoreApplication::translate("UserPointCalibrationDialog", "CALIB_USE_EXTRINSIC_GUESS", nullptr));
        QTableWidgetItem *___qtablewidgetitem = tbIntrinsic->horizontalHeaderItem(0);
        ___qtablewidgetitem->setText(QCoreApplication::translate("UserPointCalibrationDialog", "Intrinsic", nullptr));
        QTableWidgetItem *___qtablewidgetitem1 = tbIntrinsic->horizontalHeaderItem(1);
        ___qtablewidgetitem1->setText(QCoreApplication::translate("UserPointCalibrationDialog", "Standard dev.", nullptr));
        QTableWidgetItem *___qtablewidgetitem2 = tbIntrinsic->verticalHeaderItem(0);
        ___qtablewidgetitem2->setText(QCoreApplication::translate("UserPointCalibrationDialog", "fx", nullptr));
        QTableWidgetItem *___qtablewidgetitem3 = tbIntrinsic->verticalHeaderItem(1);
        ___qtablewidgetitem3->setText(QCoreApplication::translate("UserPointCalibrationDialog", "fy", nullptr));
        QTableWidgetItem *___qtablewidgetitem4 = tbIntrinsic->verticalHeaderItem(2);
        ___qtablewidgetitem4->setText(QCoreApplication::translate("UserPointCalibrationDialog", "cx", nullptr));
        QTableWidgetItem *___qtablewidgetitem5 = tbIntrinsic->verticalHeaderItem(3);
        ___qtablewidgetitem5->setText(QCoreApplication::translate("UserPointCalibrationDialog", "cy", nullptr));
        QTableWidgetItem *___qtablewidgetitem6 = tbIntrinsic->verticalHeaderItem(4);
        ___qtablewidgetitem6->setText(QCoreApplication::translate("UserPointCalibrationDialog", "k1", nullptr));
        QTableWidgetItem *___qtablewidgetitem7 = tbIntrinsic->verticalHeaderItem(5);
        ___qtablewidgetitem7->setText(QCoreApplication::translate("UserPointCalibrationDialog", "k2", nullptr));
        QTableWidgetItem *___qtablewidgetitem8 = tbIntrinsic->verticalHeaderItem(6);
        ___qtablewidgetitem8->setText(QCoreApplication::translate("UserPointCalibrationDialog", "p1", nullptr));
        QTableWidgetItem *___qtablewidgetitem9 = tbIntrinsic->verticalHeaderItem(7);
        ___qtablewidgetitem9->setText(QCoreApplication::translate("UserPointCalibrationDialog", "p2", nullptr));
        QTableWidgetItem *___qtablewidgetitem10 = tbIntrinsic->verticalHeaderItem(8);
        ___qtablewidgetitem10->setText(QCoreApplication::translate("UserPointCalibrationDialog", "k3", nullptr));
        QTableWidgetItem *___qtablewidgetitem11 = tbIntrinsic->verticalHeaderItem(9);
        ___qtablewidgetitem11->setText(QCoreApplication::translate("UserPointCalibrationDialog", "k4", nullptr));
        QTableWidgetItem *___qtablewidgetitem12 = tbIntrinsic->verticalHeaderItem(10);
        ___qtablewidgetitem12->setText(QCoreApplication::translate("UserPointCalibrationDialog", "k5", nullptr));
        QTableWidgetItem *___qtablewidgetitem13 = tbIntrinsic->verticalHeaderItem(11);
        ___qtablewidgetitem13->setText(QCoreApplication::translate("UserPointCalibrationDialog", "k6", nullptr));
        QTableWidgetItem *___qtablewidgetitem14 = tbIntrinsic->verticalHeaderItem(12);
        ___qtablewidgetitem14->setText(QCoreApplication::translate("UserPointCalibrationDialog", "s1", nullptr));
        QTableWidgetItem *___qtablewidgetitem15 = tbIntrinsic->verticalHeaderItem(13);
        ___qtablewidgetitem15->setText(QCoreApplication::translate("UserPointCalibrationDialog", "s2", nullptr));
        QTableWidgetItem *___qtablewidgetitem16 = tbIntrinsic->verticalHeaderItem(14);
        ___qtablewidgetitem16->setText(QCoreApplication::translate("UserPointCalibrationDialog", "s3", nullptr));
        QTableWidgetItem *___qtablewidgetitem17 = tbIntrinsic->verticalHeaderItem(15);
        ___qtablewidgetitem17->setText(QCoreApplication::translate("UserPointCalibrationDialog", "s4", nullptr));
        QTableWidgetItem *___qtablewidgetitem18 = tbIntrinsic->verticalHeaderItem(16);
        ___qtablewidgetitem18->setText(QCoreApplication::translate("UserPointCalibrationDialog", "taux", nullptr));
        QTableWidgetItem *___qtablewidgetitem19 = tbIntrinsic->verticalHeaderItem(17);
        ___qtablewidgetitem19->setText(QCoreApplication::translate("UserPointCalibrationDialog", "tauy", nullptr));
        QTableWidgetItem *___qtablewidgetitem20 = tbExtrinsic->horizontalHeaderItem(0);
        ___qtablewidgetitem20->setText(QCoreApplication::translate("UserPointCalibrationDialog", "Extrinsic", nullptr));
        QTableWidgetItem *___qtablewidgetitem21 = tbExtrinsic->horizontalHeaderItem(1);
        ___qtablewidgetitem21->setText(QCoreApplication::translate("UserPointCalibrationDialog", "Standard dev.", nullptr));
        QTableWidgetItem *___qtablewidgetitem22 = tbExtrinsic->verticalHeaderItem(0);
        ___qtablewidgetitem22->setText(QCoreApplication::translate("UserPointCalibrationDialog", "rx", nullptr));
        QTableWidgetItem *___qtablewidgetitem23 = tbExtrinsic->verticalHeaderItem(1);
        ___qtablewidgetitem23->setText(QCoreApplication::translate("UserPointCalibrationDialog", "ry", nullptr));
        QTableWidgetItem *___qtablewidgetitem24 = tbExtrinsic->verticalHeaderItem(2);
        ___qtablewidgetitem24->setText(QCoreApplication::translate("UserPointCalibrationDialog", "rz", nullptr));
        QTableWidgetItem *___qtablewidgetitem25 = tbExtrinsic->verticalHeaderItem(3);
        ___qtablewidgetitem25->setText(QCoreApplication::translate("UserPointCalibrationDialog", "tx", nullptr));
        QTableWidgetItem *___qtablewidgetitem26 = tbExtrinsic->verticalHeaderItem(4);
        ___qtablewidgetitem26->setText(QCoreApplication::translate("UserPointCalibrationDialog", "ty", nullptr));
        QTableWidgetItem *___qtablewidgetitem27 = tbExtrinsic->verticalHeaderItem(5);
        ___qtablewidgetitem27->setText(QCoreApplication::translate("UserPointCalibrationDialog", "tz", nullptr));
        QTableWidgetItem *___qtablewidgetitem28 = tbExtrinsic->verticalHeaderItem(6);
        ___qtablewidgetitem28->setText(QCoreApplication::translate("UserPointCalibrationDialog", "camposx", nullptr));
        QTableWidgetItem *___qtablewidgetitem29 = tbExtrinsic->verticalHeaderItem(7);
        ___qtablewidgetitem29->setText(QCoreApplication::translate("UserPointCalibrationDialog", "camposy", nullptr));
        QTableWidgetItem *___qtablewidgetitem30 = tbExtrinsic->verticalHeaderItem(8);
        ___qtablewidgetitem30->setText(QCoreApplication::translate("UserPointCalibrationDialog", "camposz", nullptr));
        QTableWidgetItem *___qtablewidgetitem31 = tbProjection->horizontalHeaderItem(0);
        ___qtablewidgetitem31->setText(QCoreApplication::translate("UserPointCalibrationDialog", "Projected xp", nullptr));
        QTableWidgetItem *___qtablewidgetitem32 = tbProjection->horizontalHeaderItem(1);
        ___qtablewidgetitem32->setText(QCoreApplication::translate("UserPointCalibrationDialog", "Projected yp", nullptr));
        QTableWidgetItem *___qtablewidgetitem33 = tbProjection->horizontalHeaderItem(2);
        ___qtablewidgetitem33->setText(QCoreApplication::translate("UserPointCalibrationDialog", "Error x (xp - xi)", nullptr));
        QTableWidgetItem *___qtablewidgetitem34 = tbProjection->horizontalHeaderItem(3);
        ___qtablewidgetitem34->setText(QCoreApplication::translate("UserPointCalibrationDialog", "Error y (yp - yi)", nullptr));
        QTableWidgetItem *___qtablewidgetitem35 = tbProjection->verticalHeaderItem(0);
        ___qtablewidgetitem35->setText(QCoreApplication::translate("UserPointCalibrationDialog", "1", nullptr));
        QTableWidgetItem *___qtablewidgetitem36 = tbProjection->verticalHeaderItem(1);
        ___qtablewidgetitem36->setText(QCoreApplication::translate("UserPointCalibrationDialog", "2", nullptr));

        const bool __sortingEnabled = tbProjection->isSortingEnabled();
        tbProjection->setSortingEnabled(false);
        tbProjection->setSortingEnabled(__sortingEnabled);

        pbUndistort->setText(QCoreApplication::translate("UserPointCalibrationDialog", "&Undistort", nullptr));
        pbLoad->setText(QCoreApplication::translate("UserPointCalibrationDialog", "&Load parameters", nullptr));
        pbSave->setText(QCoreApplication::translate("UserPointCalibrationDialog", "&Save parameters", nullptr));
        lbImgUndistorted->setText(QString());
        label->setText(QCoreApplication::translate("UserPointCalibrationDialog", "Original image", nullptr));
        pbImshow1600Undistorted->setText(QCoreApplication::translate("UserPointCalibrationDialog", "Img Show 1600", nullptr));
        label_2->setText(QCoreApplication::translate("UserPointCalibrationDialog", "Undistorted image", nullptr));
        pbImgInteractiveUndistorted->setText(QCoreApplication::translate("UserPointCalibrationDialog", "Interactive view", nullptr));
    } // retranslateUi

};

namespace Ui {
    class UserPointCalibrationDialog: public Ui_UserPointCalibrationDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_USERPOINTCALIBRATIONDIALOG_H
