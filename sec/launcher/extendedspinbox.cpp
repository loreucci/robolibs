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
