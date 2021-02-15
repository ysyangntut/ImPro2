/********************************************************************************
** Form generated from reading UI file 'QcustomplotTestDialog.ui'
**
** Created by: Qt User Interface Compiler version 5.15.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_QCUSTOMPLOTTESTDIALOG_H
#define UI_QCUSTOMPLOTTESTDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QApplication>
#include <QtWidgets/QDialog>
#include <QtWidgets/QPushButton>
#include "qcustomplot.h"

QT_BEGIN_NAMESPACE

class Ui_QcustomplotTestDialog
{
public:
    QCustomPlot *customPlot;
    QPushButton *pbBasic;
    QPushButton *pbDecaySine;
    QPushButton *pbSincScatter;
    QPushButton *pbScatterStyle;
    QPushButton *pbStyledPlot;
    QPushButton *pbColormap;
    QPushButton *pbScatterPixmap;
    QPushButton *pbTrialQcpImshow;

    void setupUi(QDialog *QcustomplotTestDialog)
    {
        if (QcustomplotTestDialog->objectName().isEmpty())
            QcustomplotTestDialog->setObjectName(QString::fromUtf8("QcustomplotTestDialog"));
        QcustomplotTestDialog->resize(1154, 629);
        customPlot = new QCustomPlot(QcustomplotTestDialog);
        customPlot->setObjectName(QString::fromUtf8("customPlot"));
        customPlot->setGeometry(QRect(20, 30, 631, 481));
        QPalette palette;
        QBrush brush(QColor(0, 0, 0, 255));
        brush.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::WindowText, brush);
        QBrush brush1(QColor(232, 211, 255, 255));
        brush1.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Button, brush1);
        QBrush brush2(QColor(255, 255, 255, 255));
        brush2.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Light, brush2);
        QBrush brush3(QColor(243, 233, 255, 255));
        brush3.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Midlight, brush3);
        QBrush brush4(QColor(116, 105, 127, 255));
        brush4.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Dark, brush4);
        QBrush brush5(QColor(155, 141, 170, 255));
        brush5.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::Mid, brush5);
        palette.setBrush(QPalette::Active, QPalette::Text, brush);
        palette.setBrush(QPalette::Active, QPalette::BrightText, brush2);
        palette.setBrush(QPalette::Active, QPalette::ButtonText, brush);
        palette.setBrush(QPalette::Active, QPalette::Base, brush2);
        palette.setBrush(QPalette::Active, QPalette::Window, brush1);
        palette.setBrush(QPalette::Active, QPalette::Shadow, brush);
        palette.setBrush(QPalette::Active, QPalette::AlternateBase, brush3);
        QBrush brush6(QColor(255, 255, 220, 255));
        brush6.setStyle(Qt::SolidPattern);
        palette.setBrush(QPalette::Active, QPalette::ToolTipBase, brush6);
        palette.setBrush(QPalette::Active, QPalette::ToolTipText, brush);
        QBrush brush7(QColor(0, 0, 0, 128));
        brush7.setStyle(Qt::SolidPattern);
#if QT_VERSION >= QT_VERSION_CHECK(5, 12, 0)
        palette.setBrush(QPalette::Active, QPalette::PlaceholderText, brush7);
#endif
        palette.setBrush(QPalette::Inactive, QPalette::WindowText, brush);
        palette.setBrush(QPalette::Inactive, QPalette::Button, brush1);
        palette.setBrush(QPalette::Inactive, QPalette::Light, brush2);
        palette.setBrush(QPalette::Inactive, QPalette::Midlight, brush3);
        palette.setBrush(QPalette::Inactive, QPalette::Dark, brush4);
        palette.setBrush(QPalette::Inactive, QPalette::Mid, brush5);
        palette.setBrush(QPalette::Inactive, QPalette::Text, brush);
        palette.setBrush(QPalette::Inactive, QPalette::BrightText, brush2);
        palette.setBrush(QPalette::Inactive, QPalette::ButtonText, brush);
        palette.setBrush(QPalette::Inactive, QPalette::Base, brush2);
        palette.setBrush(QPalette::Inactive, QPalette::Window, brush1);
        palette.setBrush(QPalette::Inactive, QPalette::Shadow, brush);
        palette.setBrush(QPalette::Inactive, QPalette::AlternateBase, brush3);
        palette.setBrush(QPalette::Inactive, QPalette::ToolTipBase, brush6);
        palette.setBrush(QPalette::Inactive, QPalette::ToolTipText, brush);
#if QT_VERSION >= QT_VERSION_CHECK(5, 12, 0)
        palette.setBrush(QPalette::Inactive, QPalette::PlaceholderText, brush7);
#endif
        palette.setBrush(QPalette::Disabled, QPalette::WindowText, brush4);
        palette.setBrush(QPalette::Disabled, QPalette::Button, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::Light, brush2);
        palette.setBrush(QPalette::Disabled, QPalette::Midlight, brush3);
        palette.setBrush(QPalette::Disabled, QPalette::Dark, brush4);
        palette.setBrush(QPalette::Disabled, QPalette::Mid, brush5);
        palette.setBrush(QPalette::Disabled, QPalette::Text, brush4);
        palette.setBrush(QPalette::Disabled, QPalette::BrightText, brush2);
        palette.setBrush(QPalette::Disabled, QPalette::ButtonText, brush4);
        palette.setBrush(QPalette::Disabled, QPalette::Base, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::Window, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::Shadow, brush);
        palette.setBrush(QPalette::Disabled, QPalette::AlternateBase, brush1);
        palette.setBrush(QPalette::Disabled, QPalette::ToolTipBase, brush6);
        palette.setBrush(QPalette::Disabled, QPalette::ToolTipText, brush);
#if QT_VERSION >= QT_VERSION_CHECK(5, 12, 0)
        palette.setBrush(QPalette::Disabled, QPalette::PlaceholderText, brush7);
#endif
        customPlot->setPalette(palette);
        pbBasic = new QPushButton(QcustomplotTestDialog);
        pbBasic->setObjectName(QString::fromUtf8("pbBasic"));
        pbBasic->setGeometry(QRect(670, 30, 93, 28));
        pbDecaySine = new QPushButton(QcustomplotTestDialog);
        pbDecaySine->setObjectName(QString::fromUtf8("pbDecaySine"));
        pbDecaySine->setGeometry(QRect(670, 70, 93, 28));
        pbSincScatter = new QPushButton(QcustomplotTestDialog);
        pbSincScatter->setObjectName(QString::fromUtf8("pbSincScatter"));
        pbSincScatter->setGeometry(QRect(670, 110, 93, 28));
        pbScatterStyle = new QPushButton(QcustomplotTestDialog);
        pbScatterStyle->setObjectName(QString::fromUtf8("pbScatterStyle"));
        pbScatterStyle->setGeometry(QRect(670, 150, 93, 28));
        pbStyledPlot = new QPushButton(QcustomplotTestDialog);
        pbStyledPlot->setObjectName(QString::fromUtf8("pbStyledPlot"));
        pbStyledPlot->setGeometry(QRect(670, 190, 93, 28));
        pbColormap = new QPushButton(QcustomplotTestDialog);
        pbColormap->setObjectName(QString::fromUtf8("pbColormap"));
        pbColormap->setGeometry(QRect(670, 230, 93, 28));
        pbScatterPixmap = new QPushButton(QcustomplotTestDialog);
        pbScatterPixmap->setObjectName(QString::fromUtf8("pbScatterPixmap"));
        pbScatterPixmap->setGeometry(QRect(670, 270, 101, 28));
        pbTrialQcpImshow = new QPushButton(QcustomplotTestDialog);
        pbTrialQcpImshow->setObjectName(QString::fromUtf8("pbTrialQcpImshow"));
        pbTrialQcpImshow->setGeometry(QRect(500, 520, 151, 28));

        retranslateUi(QcustomplotTestDialog);

        QMetaObject::connectSlotsByName(QcustomplotTestDialog);
    } // setupUi

    void retranslateUi(QDialog *QcustomplotTestDialog)
    {
        QcustomplotTestDialog->setWindowTitle(QCoreApplication::translate("QcustomplotTestDialog", "Dialog", nullptr));
        pbBasic->setText(QCoreApplication::translate("QcustomplotTestDialog", "Basic", nullptr));
        pbDecaySine->setText(QCoreApplication::translate("QcustomplotTestDialog", "Decay Sine", nullptr));
        pbSincScatter->setText(QCoreApplication::translate("QcustomplotTestDialog", "Sinc Scatter", nullptr));
        pbScatterStyle->setText(QCoreApplication::translate("QcustomplotTestDialog", "Scatter Style", nullptr));
        pbStyledPlot->setText(QCoreApplication::translate("QcustomplotTestDialog", "Styled Plot", nullptr));
        pbColormap->setText(QCoreApplication::translate("QcustomplotTestDialog", "Color Map", nullptr));
        pbScatterPixmap->setText(QCoreApplication::translate("QcustomplotTestDialog", "Scatter Pixmap", nullptr));
        pbTrialQcpImshow->setText(QCoreApplication::translate("QcustomplotTestDialog", "Trial QCP Imshow", nullptr));
    } // retranslateUi

};

namespace Ui {
    class QcustomplotTestDialog: public Ui_QcustomplotTestDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_QCUSTOMPLOTTESTDIALOG_H
