#ifndef EXTENDEDSPINBOX_H
#define EXTENDEDSPINBOX_H

#include <QWidget>
#include <QDoubleSpinBox>
#include <QPushButton>
#include <QLabel>


class ExtendedSpinBox : public QWidget {

    Q_OBJECT

public:
    explicit ExtendedSpinBox(double value = 0.0, QWidget *parent = 0);

private:
    QDoubleSpinBox* spinbox;
    QPushButton* lb, * rb;
    QLabel* scaling;
    double mult;

signals:
    void valueChanged(double);

private slots:
    void increasemult(bool);
    void decreasemult(bool);

};

#endif // EXTENDEDSPINBOX_H
