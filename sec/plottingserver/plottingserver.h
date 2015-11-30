#ifndef PLOTTINGSERVER_H
#define PLOTTINGSERVER_H

#include <QObject>
#include <QtNetwork/QLocalSocket>
#include <QtNetwork/QLocalServer>
#include <QColor>

#include "qcustomplot.h"

class PlottingServer : public QObject {

    Q_OBJECT

public:
    explicit PlottingServer(QObject* parent = 0);

signals:
    void replot();

public slots:
    void acceptConnection();
    void readMessage();

protected:
    unsigned int nextid, count;
    Qt::GlobalColor nextcolor;
    double t, dt;
    QLocalServer* server;
    QLocalSocket* socket;
    QCustomPlot* plot;

    void newplot();
    void response(const QString& response);

};

#endif // PLOTTINGSERVER_H