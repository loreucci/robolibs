#include <QApplication>

#include "launcher.h"

#include "dataoutwidget.h"
#include "extendedspinbox.h"

int main(int argc, char *argv[]) {

    QApplication a(argc, argv);

    Launcher launcher;
    launcher.show();

//    DataOutWidget widget;
//    widget.show();

//    ExtendedSpinBox widget;
//    widget.show();

    return a.exec();

}
