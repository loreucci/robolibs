#include "plottingserver.h"

#include <QApplication>

PlottingServer::PlottingServer(QObject* parent)
    :QObject(parent) {

    server = new QLocalServer(this);

    if (!server->listen("plotserver")) {
        qCritical() << "Unable to run plotting server: " << server->errorString();
        qApp->quit();
        return;
    }

    connect(server, SIGNAL(newConnection()), this, SLOT(acceptConnection()));

    newplot();

}

void PlottingServer::acceptConnection() {

    socket = server->nextPendingConnection();
    connect(socket, SIGNAL(disconnected()), socket, SLOT(deleteLater()));
    connect(socket, SIGNAL(readyRead()), this, SLOT(readMessage()));

}

Qt::GlobalColor operator++(Qt::GlobalColor& color) {
   const int i = static_cast<int>(color);
   color = static_cast<Qt::GlobalColor>((i + 1) % 20);
   return color;
}

void PlottingServer::readMessage() {

    QDataStream in(socket);

    QString nextMsg;
    in >> nextMsg;

    QStringList cmd = nextMsg.split(" ");

//    qDebug() << nextMsg;

    if (cmd[0] == "addgraph") {             // add new graph

        if (cmd.size() < 2) {
            response("ERROR");
            return;
        }
        response(QString::number(nextid));

        plot->addGraph();
        plot->graph(nextid)->setPen(QPen(nextcolor, 2));
        plot->graph(nextid)->setName(cmd[1]);

        ++nextcolor;
        ++nextid;


    } else if (cmd[0] == "newvalues") {      // add new values to the plots

        if (cmd.size() != plot->plottableCount()*2+1) {
            response("ERROR");
            return;
        }
        response("OK");

        for (unsigned int i = 1; i < cmd.size(); i+=2){
            int id = cmd[i].toInt();
            plot->graph(id)->addData(t, cmd[i+1].toDouble());
            plot->graph(id)->rescaleValueAxis();
        }




    }/*else if (cmd[0] == "newvalue") {      // add new value to a plot

        if (cmd.size() < 3) {
            response("ERROR");
            return;
        }
        response("OK");

        int id = cmd[1].toInt();

        plot->graph(id)->addData(t, cmd[2].toDouble());
        plot->graph(id)->rescaleValueAxis();


    }*/ else if (cmd[0] == "clearall") {      // reset plot

        response("OK");

        plot->deleteLater();
        newplot();


    } else if (cmd[0] == "setfreq") {       // change sampling freq

        response("OK");

        if (cmd.size() < 2) {
            response("ERROR");
            return;
        }

        dt = 1/cmd[1].toDouble();


    } else if (cmd[0] == "advance") {       // advance plot

        response("OK");

        plot->xAxis->setRange(t, 5, Qt::AlignRight);
        t += dt;
        emit replot();


    } else {                                // error

        qWarning() << "Unknown command: " << cmd[0];
        response("ERROR");

    }

}

void PlottingServer::newplot() {

    nextid = 0;
    nextcolor = Qt::red;
    t = 0;
    dt = 0.01;
    count = 0;

    this->plot = new QCustomPlot();
    this->plot->resize(800, 600);

    this->plot->legend->setVisible(true);

    // configure right and top axis to show ticks but no labels:
    this->plot->xAxis2->setVisible(true);
    this->plot->xAxis2->setTickLabels(false);
    this->plot->yAxis2->setVisible(true);
    this->plot->yAxis2->setTickLabels(false);

    // set tick steps
    plot->xAxis->setAutoTickStep(false);
    plot->xAxis->setTickStep(0.5);

    // make left and bottom axes always transfer their ranges to right and top axes:
    connect(this->plot->xAxis, SIGNAL(rangeChanged(QCPRange)), this->plot->xAxis2, SLOT(setRange(QCPRange)));
    connect(this->plot->yAxis, SIGNAL(rangeChanged(QCPRange)), this->plot->yAxis2, SLOT(setRange(QCPRange)));
    //        app.connect(this, SIGNAL(advance_sig()), this, SLOT(advance_slot()));

    // asynchronous replot
    connect(this, SIGNAL(replot()), plot, SLOT(replot()));

    // Note: we could have also just called customPlot->rescaleAxes(); instead
    // Allow user to drag axis ranges with mouse, zoom with mouse wheel and select graphs by clicking:
    this->plot->setInteractions(QCP::iRangeDrag | QCP::iRangeZoom | QCP::iSelectPlottables);

    this->plot->replot();

    this->plot->show();

}

void PlottingServer::response(const QString& response) {

    QByteArray msg;
    QDataStream out(&msg, QIODevice::WriteOnly);
    out << response;

    socket->write(msg);
    socket->flush();

}
