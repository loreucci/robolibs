#ifndef PID_H
#define PID_H

#include <sec/node.h>
#include <sec/nodelink.h>


class PID : public sec::Node {

public:
    PID(double kp, double ki, double kd, double freq = 0.0);

    virtual void refreshInputs();

    virtual bool connected() const;

    virtual void execute();

    virtual std::string parameters() const;

    // inputs
    sec::NodeIn<double> ref;
    sec::NodeIn<double> enc;

    // output
    sec::NodeOut<double> output;

protected:
    double kp, ki, kd;
    double preverr, erri;

};

#endif // PID_H
