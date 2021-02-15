/********************************************************************************
** Form generated from reading UI file 'ConsoleG.ui'
**
** Created by: Qt User Interface Compiler version 5.15.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_CONSOLEG_H
#define UI_CONSOLEG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QStatusBar>
#include <QtWidgets/QWidget>

QT_BEGIN_NAMESPACE

class Ui_ConsoleG
{
public:
    QAction *action_Camera_Add_ByUserPoints;
    QAction *action_Tests_QcustomplotBasic;
    QWidget *centralwidget;
    QMenuBar *menubar;
    QMenu *menu_Project;
    QMenu *menu_Image_Sequence;
    QMenu *menu_Camear;
    QMenu *menu_Add;
    QMenu *menu_Target;
    QMenu *menuC_urve;
    QMenu *menu_Surface;
    QMenu *menu_About;
    QMenu *menutests;
    QStatusBar *statusbar;

    void setupUi(QMainWindow *ConsoleG)
    {
        if (ConsoleG->objectName().isEmpty())
            ConsoleG->setObjectName(QString::fromUtf8("ConsoleG"));
        ConsoleG->resize(1204, 649);
        action_Camera_Add_ByUserPoints = new QAction(ConsoleG);
        action_Camera_Add_ByUserPoints->setObjectName(QString::fromUtf8("action_Camera_Add_ByUserPoints"));
        action_Tests_QcustomplotBasic = new QAction(ConsoleG);
        action_Tests_QcustomplotBasic->setObjectName(QString::fromUtf8("action_Tests_QcustomplotBasic"));
        centralwidget = new QWidget(ConsoleG);
        centralwidget->setObjectName(QString::fromUtf8("centralwidget"));
        ConsoleG->setCentralWidget(centralwidget);
        menubar = new QMenuBar(ConsoleG);
        menubar->setObjectName(QString::fromUtf8("menubar"));
        menubar->setGeometry(QRect(0, 0, 1204, 26));
        menu_Project = new QMenu(menubar);
        menu_Project->setObjectName(QString::fromUtf8("menu_Project"));
        menu_Image_Sequence = new QMenu(menubar);
        menu_Image_Sequence->setObjectName(QString::fromUtf8("menu_Image_Sequence"));
        menu_Camear = new QMenu(menubar);
        menu_Camear->setObjectName(QString::fromUtf8("menu_Camear"));
        menu_Add = new QMenu(menu_Camear);
        menu_Add->setObjectName(QString::fromUtf8("menu_Add"));
        menu_Target = new QMenu(menubar);
        menu_Target->setObjectName(QString::fromUtf8("menu_Target"));
        menuC_urve = new QMenu(menubar);
        menuC_urve->setObjectName(QString::fromUtf8("menuC_urve"));
        menu_Surface = new QMenu(menubar);
        menu_Surface->setObjectName(QString::fromUtf8("menu_Surface"));
        menu_About = new QMenu(menubar);
        menu_About->setObjectName(QString::fromUtf8("menu_About"));
        menutests = new QMenu(menubar);
        menutests->setObjectName(QString::fromUtf8("menutests"));
        ConsoleG->setMenuBar(menubar);
        statusbar = new QStatusBar(ConsoleG);
        statusbar->setObjectName(QString::fromUtf8("statusbar"));
        ConsoleG->setStatusBar(statusbar);

        menubar->addAction(menu_Project->menuAction());
        menubar->addAction(menu_Image_Sequence->menuAction());
        menubar->addAction(menu_Camear->menuAction());
        menubar->addAction(menu_Target->menuAction());
        menubar->addAction(menuC_urve->menuAction());
        menubar->addAction(menu_Surface->menuAction());
        menubar->addAction(menu_About->menuAction());
        menubar->addAction(menutests->menuAction());
        menu_Camear->addAction(menu_Add->menuAction());
        menu_Add->addAction(action_Camera_Add_ByUserPoints);
        menutests->addAction(action_Tests_QcustomplotBasic);

        retranslateUi(ConsoleG);

        QMetaObject::connectSlotsByName(ConsoleG);
    } // setupUi

    void retranslateUi(QMainWindow *ConsoleG)
    {
        ConsoleG->setWindowTitle(QCoreApplication::translate("ConsoleG", "ConsoleG", nullptr));
        action_Camera_Add_ByUserPoints->setText(QCoreApplication::translate("ConsoleG", "by &user points", nullptr));
#if QT_CONFIG(shortcut)
        action_Camera_Add_ByUserPoints->setShortcut(QCoreApplication::translate("ConsoleG", "Ctrl+U", nullptr));
#endif // QT_CONFIG(shortcut)
        action_Tests_QcustomplotBasic->setText(QCoreApplication::translate("ConsoleG", "QcustomplotBasic", nullptr));
#if QT_CONFIG(shortcut)
        action_Tests_QcustomplotBasic->setShortcut(QCoreApplication::translate("ConsoleG", "Ctrl+Shift+T", nullptr));
#endif // QT_CONFIG(shortcut)
        menu_Project->setTitle(QCoreApplication::translate("ConsoleG", "&Project", nullptr));
        menu_Image_Sequence->setTitle(QCoreApplication::translate("ConsoleG", "&Image Sequence", nullptr));
        menu_Camear->setTitle(QCoreApplication::translate("ConsoleG", "&Camera", nullptr));
        menu_Add->setTitle(QCoreApplication::translate("ConsoleG", "&Add", nullptr));
        menu_Target->setTitle(QCoreApplication::translate("ConsoleG", "&Target", nullptr));
        menuC_urve->setTitle(QCoreApplication::translate("ConsoleG", "C&urve", nullptr));
        menu_Surface->setTitle(QCoreApplication::translate("ConsoleG", "&Surface", nullptr));
        menu_About->setTitle(QCoreApplication::translate("ConsoleG", "&About", nullptr));
        menutests->setTitle(QCoreApplication::translate("ConsoleG", "tests", nullptr));
    } // retranslateUi

};

namespace Ui {
    class ConsoleG: public Ui_ConsoleG {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_CONSOLEG_H
