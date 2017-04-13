#include <QApplication>

#include "launcher.h"


int main(int argc, char *argv[]) {

    QApplication a(argc, argv);

    Launcher launcher;

    launcher.show();

    return a.exec();

}
