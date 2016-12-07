#include "signals.h"

#include <cmath>
#include "utilities.h"

#include <iostream>


namespace Signals {


class SignalRuntime {

public:
    SignalRuntime() {
        signals_allocated.clear();
    }

    ~SignalRuntime() {
//        std::cout << "Runtime destructor: " << signals_allocated.size() << std::endl;
        for (auto& s : signals_allocated)
            delete s;
    }

//    void deleteSignal(Signal* s) {
//        for (auto it = signals_allocated.begin(); it != signals_allocated.end(); ++it) {
//            if (*it == s) {
//                delete s;
//                signals_allocated.erase(it);
//                break;
//            }
//        }
//        std::cout << "Runtime delete: " << signals_allocated.size() << std::endl;
//    }

    void addSignal(Signal* s) {
        signals_allocated.push_back(s);
//        std::cout << "Runtime add: " << signals_allocated.size() << std::endl;
    }

protected:
    std::vector<Signal*> signals_allocated;

};

SignalRuntime signal_runtime;


Signal::Signal(double samplingfreq) {

    if (samplingfreq < 0.0)
        throw std::invalid_argument("Signal: sampling frequency must be non-negative.");
    this->samplingfreq = samplingfreq;
    t = 0;

}

Signal::~Signal() {}

double Signal::operator()() {
    return output();
}

void Signal::reset(double time) {

    t = time * samplingfreq;

}

double Signal::getSamplingFreq() const {
    return samplingfreq;
}

void Signal::setSamplingFreq(double samplingfreq) {
    if (samplingfreq < 0.0)
        throw std::invalid_argument("Signal: sampling frequency must be non-negative.");
    this->samplingfreq = samplingfreq;
}



sin::sin(double ampl, double freq, double phase, double samplingfreq)
    :Signal(samplingfreq), ampl(ampl), freq(freq), phase(phase) {}

double sin::output() {
    double ret = ampl * std::sin(2*Utils::PI*freq*t/samplingfreq + phase);
    t++;
    return ret;
}

std::string sin::to_string() const {

    std::string str = "[sin: ";
    str += "a=" + std::to_string(ampl) + ", ";
    str += "f=" + std::to_string(freq) + ", ";
    str += "ph=" + std::to_string(phase) + "]";
    return str;

}



constant::constant(double c, double samplingfreq)
    :Signal(samplingfreq), c(c) {}

double constant::output() {
    return c;
}

std::string constant::to_string() const {
    return "[constant: " + std::to_string(c) + "]";
}



ramp::ramp(double slope, double initialvalue, double starttime, double samplingfreq)
    :Signal(samplingfreq), slope(slope), initialvalue(initialvalue), starttime(starttime) {}

double ramp::output() {
    double time = t / samplingfreq;
    if (time <= starttime) {
        t++;
        return initialvalue;
    }
    t++;
    return slope*(time-starttime);
}

std::string ramp::to_string() const {
    std::string str = "[ramp: ";
    str += "slope=" + std::to_string(slope) + ", ";
    str += "initialvalue=" + std::to_string(initialvalue) + ", ";
    str += "starttime=" + std::to_string(starttime) + "]";
    return str;
}



rampandhold::rampandhold(double slope, double initialvalue, double stoptime, double starttime, double samplingfreq)
    :Signal(samplingfreq), slope(slope), initialvalue(initialvalue), stoptime(stoptime), starttime(starttime) {
    lastvalue = 0.0;
}

double rampandhold::output() {
    double time = t / samplingfreq;
    if (time <= starttime)
        lastvalue = initialvalue;
    else if (time <= stoptime)
        lastvalue = slope*(time-starttime);
    t++;
    return lastvalue;
}

std::string rampandhold::to_string() const {
    std::string str = "[ramp: ";
    str += "slope=" + std::to_string(slope) + ", ";
    str += "initialvalue=" + std::to_string(initialvalue) + ", ";
    str += "stoptime=" + std::to_string(stoptime) + ", ";
    str += "starttime=" + std::to_string(starttime) + "]";
    return str;
}



Switch::Switch(Signal& s1, Signal& s2, double switchtime, double samplingfreq)
    :Signal(samplingfreq), s1(s1), s2(s2), switchtime(switchtime) {}

double Switch::output() {

    double out1 = s1();
    double out2 = s2();

    if (t/samplingfreq <= switchtime) {
        t++;
        return out1;
    }
    t++;
    return out2;

}

std::string Switch::to_string() const {
    return "[switch: s1=" + s1.to_string() + ", s2=" + s2.to_string() + ", time=" + std::to_string(switchtime) + "]";
}

void Switch::reset(double time) {
    s1.reset(time);
    s2.reset(time);
}

void Switch::setSamplingFreq(double samplingfreq) {
    Signal::setSamplingFreq(samplingfreq);
    s1.setSamplingFreq(samplingfreq);
    s2.setSamplingFreq(samplingfreq);
}



BinaryOperation::BinaryOperation(Signal& s1, Signal& s2, std::function<double(double, double)> fun)
    :Signal(0.0), s1(s1), s2(s2), fun(fun) {}

BinaryOperation::~BinaryOperation() {
//    signal_runtime.deleteSignal(&s1);
//    signal_runtime.deleteSignal(&s2);
}

//BinaryOperation::BinaryOperation(Signal& s1, Signal&& s2, std::function<double(double, double)> fun)
//    :Signal(0.0), s1(s1), s2(s2), fun(fun) {}

//BinaryOperation::BinaryOperation(Signal&& s1, Signal& s2, std::function<double(double, double)> fun)
//    :Signal(0.0), s1(s1), s2(s2), fun(fun) {}

//BinaryOperation::BinaryOperation(Signal&& s1, Signal&& s2, std::function<double(double, double)> fun)
//    :Signal(0.0), s1(s1), s2(s2), fun(fun) {}

double BinaryOperation::output() {
    return fun(s1(), s2());
}

std::string BinaryOperation::to_string() const {
    return "[operation: s1=" + s1.to_string() + ", s2=" + s2.to_string() + "]";
}

void BinaryOperation::reset(double time) {
    s1.reset(time);
    s2.reset(time);
}

void BinaryOperation::setSamplingFreq(double samplingfreq) {
    Signal::setSamplingFreq(samplingfreq);
    s1.setSamplingFreq(samplingfreq);
    s2.setSamplingFreq(samplingfreq);
}

BinaryOperation operator+(Signal& s1, Signal& s2) {
    return BinaryOperation(s1, s2, std::plus<double>());
}

BinaryOperation operator-(Signal& s1, Signal& s2) {
    return BinaryOperation(s1, s2, std::minus<double>());
}

BinaryOperation operator*(Signal& s1, Signal& s2) {
    return BinaryOperation(s1, s2, std::multiplies<double>());
}

BinaryOperation operator/(Signal& s1, Signal& s2) {
    return BinaryOperation(s1, s2, std::divides<double>());
}


BinaryOperation operator+(double c, Signal& s) {
    Signal* cs = new constant(c, s.getSamplingFreq());
    signal_runtime.addSignal(cs);
    return BinaryOperation(*cs, s, std::plus<double>());
}

BinaryOperation operator+(double c, Signal&& s) {
    Signal* cs = new constant(c, s.getSamplingFreq());
    signal_runtime.addSignal(cs);
    return BinaryOperation(*cs, s, std::plus<double>());
}

BinaryOperation operator+(Signal& s, double c) {
    Signal* cs = new constant(c, s.getSamplingFreq());
    signal_runtime.addSignal(cs);
    return BinaryOperation(s, *cs, std::plus<double>());
}

BinaryOperation operator+(Signal&& s, double c) {
    Signal* cs = new constant(c, s.getSamplingFreq());
    signal_runtime.addSignal(cs);
    return BinaryOperation(s, *cs, std::plus<double>());
}


BinaryOperation operator-(double c, Signal& s) {
    Signal* cs = new constant(c, s.getSamplingFreq());
    signal_runtime.addSignal(cs);
    return BinaryOperation(*cs, s, std::minus<double>());
}

BinaryOperation operator-(double c, Signal&& s) {
    Signal* cs = new constant(c, s.getSamplingFreq());
    signal_runtime.addSignal(cs);
    return BinaryOperation(*cs, s, std::minus<double>());
}

BinaryOperation operator-(Signal& s, double c) {
    Signal* cs = new constant(c, s.getSamplingFreq());
    signal_runtime.addSignal(cs);
    return BinaryOperation(s, *cs, std::minus<double>());
}

BinaryOperation operator-(Signal&& s, double c) {
    Signal* cs = new constant(c, s.getSamplingFreq());
    signal_runtime.addSignal(cs);
    return BinaryOperation(s, *cs, std::minus<double>());
}


BinaryOperation operator*(double c, Signal& s) {
    Signal* cs = new constant(c, s.getSamplingFreq());
    signal_runtime.addSignal(cs);
    return BinaryOperation(*cs, s, std::multiplies<double>());
}

BinaryOperation operator*(double c, Signal&& s) {
    Signal* cs = new constant(c, s.getSamplingFreq());
    signal_runtime.addSignal(cs);
    return BinaryOperation(*cs, s, std::multiplies<double>());
}

BinaryOperation operator*(Signal& s, double c) {
    Signal* cs = new constant(c, s.getSamplingFreq());
    signal_runtime.addSignal(cs);
    return BinaryOperation(s, *cs, std::multiplies<double>());
}

BinaryOperation operator*(Signal&& s, double c) {
    Signal* cs = new constant(c, s.getSamplingFreq());
    signal_runtime.addSignal(cs);
    return BinaryOperation(s, *cs, std::multiplies<double>());
}


BinaryOperation operator/(double c, Signal& s) {
    Signal* cs = new constant(c, s.getSamplingFreq());
    signal_runtime.addSignal(cs);
    return BinaryOperation(*cs, s, std::divides<double>());
}

BinaryOperation operator/(double c, Signal&& s) {
    Signal* cs = new constant(c, s.getSamplingFreq());
    signal_runtime.addSignal(cs);
    return BinaryOperation(*cs, s, std::divides<double>());
}

BinaryOperation operator/(Signal& s, double c) {
    Signal* cs = new constant(c, s.getSamplingFreq());
    signal_runtime.addSignal(cs);
    return BinaryOperation(s, *cs, std::divides<double>());
}

BinaryOperation operator/(Signal&& s, double c) {
    Signal* cs = new constant(c, s.getSamplingFreq());
    signal_runtime.addSignal(cs);
    return BinaryOperation(s, *cs, std::divides<double>());
}


//BinaryOperation operator+(Signal& s1, double s2) {
//    return BinaryOperation(s1, constant(s2), std::plus<double>());
//}

//BinaryOperation operator+(double s1, Signal& s2) {
//    BinaryOperation(constant(s1), s2, std::plus<double>());
//}


}
