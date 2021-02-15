#ifndef QCUSTOMPLOTTESTDIALOG_H
#define QCUSTOMPLOTTESTDIALOG_H

#include <QDialog>

namespace Ui {
class QcustomplotTestDialog;
}

class QcustomplotTestDialog : public QDialog
{
    Q_OBJECT

public:
    explicit QcustomplotTestDialog(QWidget *parent = nullptr);
    ~QcustomplotTestDialog();

private slots:
    void on_pbBasic_clicked();

    void on_pbDecaySine_clicked();

    void on_pbSincScatter_clicked();

    void on_pbScatterStyle_clicked();

    void on_pbStyledPlot_clicked();

    void on_pbColormap_clicked();

    void on_pbScatterPixmap_clicked();

    void on_pbTrialQcpImshow_clicked();

private:
    Ui::QcustomplotTestDialog *ui;
};

#endif // QCUSTOMPLOTTESTDIALOG_H
