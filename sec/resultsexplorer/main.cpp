#include <QApplication>

#include "resultsexplorer.h"


int main(int argc, char *argv[]) {

    QApplication a(argc, argv);

    ResultsExplorer resexplorer;

    resexplorer.show();

    return a.exec();

}
