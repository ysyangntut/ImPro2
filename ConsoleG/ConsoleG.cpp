#include "ConsoleG.h"
#include "ui_ConsoleG.h"

#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <cstdio>
#include <ctime>
#include <thread>
#include <chrono>
using std::cout;
using std::endl;
using std::vector;
using std::string;

#include "UserPointCalibrationDialog.h"


#include "QcustomplotTestDialog.h"



ConsoleG::ConsoleG(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::ConsoleG)
{
    ui->setupUi(this);
}

ConsoleG::~ConsoleG()
{
    delete ui;
}


void ConsoleG::on_action_Tests_QcustomplotBasic_triggered()
{
    QcustomplotTestDialog dialog;
    dialog.setWindowTitle("QCustomPlot Test");
    dialog.exec();
}

void ConsoleG::on_action_Camera_Add_ByUserPoints_triggered()
{
    UserPointCalibrationDialog dialog;
    dialog.setWindowTitle("User-Point Calibration");
    dialog.exec();
}
