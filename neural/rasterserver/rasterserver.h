/*
 * Copyright (C) 2014-2019 Lorenzo Vannucci
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef PLOTTINGSERVER_H
#define PLOTTINGSERVER_H

#include <QObject>
#include <QtNetwork/QLocalSocket>
#include <QtNetwork/QLocalServer>
#include <QColor>

#include <qcustomplot/qcustomplot.h>

class RasterServer : public QObject {

    Q_OBJECT

public:
    explicit RasterServer(QObject* parent = 0);

signals:
    void replot();

public slots:
    void acceptConnection();
    void readMessage();

protected:
    unsigned int nextid;
    Qt::GlobalColor nextcolor;
    double t, dt;
    QLocalServer* server;
    QLocalSocket* socket;
    QCustomPlot* plot;

    int maxneuronid, minneuronid;

    void newplot();
    void response(const QString& response);

};

#endif // PLOTTINGSERVER_H
