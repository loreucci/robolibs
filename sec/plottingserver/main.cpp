#include <QApplication>

#include "plottingserver.h"

int main(int argc, char *argv[]) {

    QApplication a(argc, argv);

    PlottingServer plottingsrv(&a);

    return a.exec();
}
