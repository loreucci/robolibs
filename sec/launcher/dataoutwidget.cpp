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

#include "dataoutwidget.h"

#include <QGridLayout>
#include <QBoxLayout>
#include <QLabel>
#include <cmath>


ExpValue::ExpValue(std::string name, double val, sec::NodeIn<double>* inlink, QWidget* parentw, QObject* parentobj)
    :QObject(parentobj), name(name), inlink(inlink) {

    check = new QCheckBox(parentw);
    check->setCheckState(Qt::Unchecked);

    label = new QLabel(QString::fromStdString(name)+":", parentw);

    spinbox = new ExtendedSpinBox(val, parentw);

    outlink = val;

    checked(Qt::Unchecked);

    connect(check, SIGNAL(stateChanged(int)), this, SLOT(checked(int)));
    connect(spinbox, SIGNAL(valueChanged(double)), this, SLOT(valueChanged(double)));

}

void ExpValue::checked(int state) {

    if (state == Qt::Unchecked) {
//        label->setEnabled(false);
//        spinbox->setEnabled(false);
        inlink->connect(nullptr);
    } else {
//        label->setEnabled(true);
//        spinbox->setEnabled(true);
        inlink->connect(&outlink);
    }

}

void ExpValue::valueChanged(double val) {
    outlink = val;
}


DataOutWidget::DataOutWidget(QString socketname, QWidget *parent)
    :QWidget(parent) {


    dataout = new sec::SocketDataOut(socketname.toStdString());
    dataout->waitForServer();

    QBoxLayout* mainl = new QBoxLayout(QBoxLayout::TopToBottom, this);

    // frequency row
    QWidget* freqw = new QWidget(this);
    QBoxLayout* freql = new QBoxLayout(QBoxLayout::LeftToRight, freqw);
    freql->addWidget(new QLabel("Data frequency:", freqw));
    freqe = new QLineEdit("1", freqw);
    connect(freqe, SIGNAL(textEdited(QString)), this, SLOT(timerchanged(QString)));
    freql->addWidget(freqe);
    freql->addWidget(new QLabel("Hz", freqw));
    freqw->setLayout(freql);


    // data rows
    QWidget* dataw = new QWidget(this);
    QGridLayout* datal = new QGridLayout(dataw);
    auto explist = dataout->getExpected();
    int i = 0;
    for (auto& exp : explist) {

        ExpValue* expvalue = new ExpValue(exp.first, exp.second, &dataout->input(exp.first), this, this);
        datal->addWidget(expvalue->check, i, 0);
        datal->addWidget(expvalue->label, i, 1);
        datal->addWidget(expvalue->spinbox, i, 2);
        expected.append(expvalue);

        i++;

    }
    dataw->setLayout(datal);


    mainl->addWidget(freqw);
    mainl->addWidget(dataw);
    this->setLayout(mainl);

    timer = new QTimer(this);
    connect(timer, SIGNAL(timeout()), this, SLOT(timerexpired()));
    timer->start(1000);

    dataout->sendStart();

}

DataOutWidget::~DataOutWidget() {
    timer->stop();
    dataout->deleteLater();
    dataout = nullptr;
}

void DataOutWidget::timerexpired() {

    dataout->refreshInputs();
    dataout->execute();

}

void DataOutWidget::timerchanged(QString rate) {

    bool ok;
    int wait = std::round(1000.0/rate.toDouble(&ok));
    if (ok && wait > 0) {
        timer->stop();
        timer->start(wait);
    }

}
