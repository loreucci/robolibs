#ifndef SIMPLESOURCES_H
#define SIMPLESOURCES_H

#include <utilities/utilities.h>

#include "source.h"
#include "nodelink.h"


namespace sec {

template <typename T>
class ConstantSource  : public Source {

public:
    ConstantSource(const T& value = T(), double freq = 0.0)
        :Source(freq), value(value) {}

    virtual void execute() override {
        output.addData(value);
    }

    virtual std::string parameters() const override {
        return "ConstantSource of value" + Utils::make_string(value, " ") + ".";
    }

    NodeOut<T> output;

protected:
    T value;

};


class SinusoidalSource : public Source {

public:
    SinusoidalSource(double ampl, double freq, double phase = 0.0, double mean = 0.0, double samplingfreq = 0.0);

    virtual void execute() override;

    virtual std::string parameters() const override;

    NodeOut<double> output;

protected:
    double ampl, freq, phase, mean;
    unsigned int step;

};

}

#endif // SIMPLESOURCES_H
