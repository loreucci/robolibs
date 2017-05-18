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
