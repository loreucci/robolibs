/*
 * Copyright (C) 2014-2019 Lorenzo Vannucci
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

#ifndef DATAOUTWIDGET_H
#define DATAOUTWIDGET_H

#include <QWidget>
#include <QCheckBox>
#include <QLineEdit>
#include <QString>
#include <QList>
#include <QTimer>

#include "../sec/nodelink.h"
#include "../sec/socketdataexchange.h"

#include "extendedspinbox.h"


class ExpValue : public QObject {

    Q_OBJECT

public:
    ExpValue(std::string name, double val, sec::NodeIn<double>* inlink, QWidget* parentw, QObject* parentobj = nullptr);

    QCheckBox* check;
    std::string name;
    QLabel* label;
    ExtendedSpinBox* spinbox;
    sec::NodeOut<double> outlink;
    sec::NodeIn<double>* inlink;

public slots:
    void checked(int state);
    void valueChanged(double val);

};



class DataOutWidget : public QWidget {

    Q_OBJECT

public:
    explicit DataOutWidget(QString socketname, QWidget* parent = nullptr);
    virtual ~DataOutWidget();

protected:
    sec::SocketDataOut* dataout;
    QList<ExpValue*> expected;

    QLineEdit* freqe;
    QTimer* timer;
    unsigned int count = 0;

private slots:
    void timerexpired();
    void timerchanged(QString rate);

};

#endif // DATAOUTWIDGET_H
