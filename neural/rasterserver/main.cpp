#include <QApplication>

#include "rasterserver.h"

int main(int argc, char *argv[]) {

    QApplication a(argc, argv);

    RasterServer plottingsrv(&a);

    return a.exec();
}
