#ifndef CONSOLEG_H
#define CONSOLEG_H

#include <QMainWindow>

QT_BEGIN_NAMESPACE
namespace Ui { class ConsoleG; }
QT_END_NAMESPACE

class ConsoleG : public QMainWindow
{
    Q_OBJECT

public:
    ConsoleG(QWidget *parent = nullptr);
    ~ConsoleG();

private slots:
    void on_action_Tests_QcustomplotBasic_triggered();


    void on_action_Camera_Add_ByUserPoints_triggered();

private:
    Ui::ConsoleG *ui;
};
#endif // CONSOLEG_H
