#include "ConsoleG.h"

#include <QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    ConsoleG w;
    w.show();
    return a.exec();
}
