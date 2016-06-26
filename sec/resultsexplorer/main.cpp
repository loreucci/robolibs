#include <QApplication>

//#include <QtSql/QSqlDatabase>
//#include <QMessageBox>
//#include <QtSql/QSqlQuery>
//#include <QDebug>
//#include <QFile>

#include "resultsexplorer.h"


//void createSampleDatabase() {

//    if (QFile::exists("logs.sqlite")) {
//        qDebug() << "Database already exists.\n";
//        return;
//    }

//    // http://cep.xray.aps.anl.gov/software/qt4-x11-4.2.2-browser/df/d3c/sql_2connection_8h-source.html
//    QSqlDatabase db = QSqlDatabase::addDatabase("QSQLITE", "database");
//    db.setDatabaseName("logs.sqlite");
//    if (!db.open()) {
//        QMessageBox::critical(0, qApp->tr("Cannot open database"),
//            qApp->tr("Unable to establish a database connection.\n"
//                     "This example needs SQLite support. Please read "
//                     "the Qt SQL driver documentation for information how "
//                     "to build it.\n\n"
//                     "Click Cancel to exit."), QMessageBox::Cancel);
//        return;
//    }

//    QSqlQuery query;
//    if (!query.exec("CREATE TABLE logs (name TEXT NOT NULL, params INTEGER NOT NULL,"
//                    "comment TEXT, expname TEXT);"))
//        qDebug() << "wrong query1\n";
//    if (!query.exec("INSERT INTO logs values('test2016.txt', 1, NULL, NULL);"))
//        qDebug() << "wrong query2\n";
//    if (!query.exec("INSERT INTO logs values('test201605171610.txt', 1, 'Nice', 'working.txt');"))
//        qDebug() << "wrong query3\n";

//    db.close();
//}


int main(int argc, char *argv[]) {

//    createSampleDatabase();

    QApplication a(argc, argv);

    ResultsExplorer resexplorer;

    resexplorer.show();

    return a.exec();

}
