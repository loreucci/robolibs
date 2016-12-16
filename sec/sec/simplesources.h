#ifndef SIMPLESOURCES_H
#define SIMPLESOURCES_H

#include <fstream>
#include <vector>

#include <utilities/utilities.h>
#include <utilities/signals.h>
#include <utilities/vector.h>

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


class [[deprecated("Replaced by SignalSource.")]] SinusoidalSource : public Source {

public:
    SinusoidalSource(double ampl, double freq, double phase = 0.0, double mean = 0.0, double samplingfreq = 0.0);

    virtual void execute() override;

    virtual std::string parameters() const override;

    NodeOut<double> output;

protected:
    double ampl, freq, phase, mean;
    unsigned int step;

};


class SignalSource : public Source {

public:
    SignalSource(Signals::Signal signal, double samplingfreq = 0.0);

    virtual void execute() override;

    virtual std::string parameters() const override;

    NodeOut<double> output;

protected:
    Signals::Signal signal;

};

class SignalSourceVector : public Source {

public:
    SignalSourceVector(const std::vector<Signals::Signal>& signalvec, double samplingfreq = 0.0);

    virtual void execute() override;

    virtual std::string parameters() const override;

    NodeOut<Utils::Vector> output;

protected:
    std::vector<Signals::Signal> signalvec;

};


class FileSource : public Source {

public:
    FileSource(const std::string& filename, unsigned int ignorelines = 0, bool repeat = false, double samplingfreq = 0.0);

    virtual void execute() override;

    virtual std::string parameters() const override;

    bool valid() const;

    NodeOut<std::vector<double>> output;

protected:
    std::string filename;
    unsigned int ignorelines;
    bool repeat, over;
    std::ifstream file;

    void skiplines();

};

}


#endif // SIMPLESOURCES_H
