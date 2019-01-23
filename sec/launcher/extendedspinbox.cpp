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

#include "extendedspinbox.h"

#include <QBoxLayout>
#include <limits>

ExtendedSpinBox::ExtendedSpinBox(double value, QWidget *parent)
    :QWidget(parent) {

    QBoxLayout* layout = new QBoxLayout(QBoxLayout::LeftToRight, this);

    mult = 1.0;

    // spinbox
    spinbox = new QDoubleSpinBox(this);
    spinbox->setMaximum(std::numeric_limits<double>::max());
    spinbox->setMinimum(std::numeric_limits<double>::lowest());
    spinbox->setDecimals(5);
    spinbox->setValue(value);
    connect(spinbox, SIGNAL(valueChanged(double)), this, SIGNAL(valueChanged(double)));

    // increase button
    layout->addWidget(spinbox, 6);
    lb = new QPushButton("<", this);
    lb->setMaximumWidth(lb->fontMetrics().boundingRect("<").width()+7);
    connect(lb, SIGNAL(clicked(bool)), this, SLOT(increasemult(bool)));
    layout->addWidget(lb, 1);

    // scaling label
    scaling = new QLabel(QString::number(mult), this);
    layout->addWidget(scaling, 2);

    // decreasebutton
    rb = new QPushButton(">", this);
    rb->setMaximumWidth(rb->fontMetrics().boundingRect(">").width()+7);
    connect(rb, SIGNAL(clicked(bool)), this, SLOT(decreasemult(bool)));
    layout->addWidget(rb, 1);

    this->setLayout(layout);

}

void ExtendedSpinBox::increasemult(bool) {
    mult *= 10.0;
    spinbox->setSingleStep(mult);
    scaling->setText(QString::number(mult));
}

void ExtendedSpinBox::decreasemult(bool) {
    mult /= 10.0;
    spinbox->setSingleStep(mult);
    scaling->setText(QString::number(mult));
}
