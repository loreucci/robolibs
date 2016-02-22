#include "signals.h"

#include <cmath>
#include "utilities.h"


namespace Signals {


Signal::Signal(double samplingfreq) {

    if (samplingfreq < 0.0)
        throw std::invalid_argument("Signal: sampling frequency must be non-negative.");
    this->samplingfreq = samplingfreq;
    t = 0;

}

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



BinaryOperation::BinaryOperation(Signal& s1, Signal& s2, std::function<double(double, double)> fun)
    :Signal(0.0), s1(s1), s2(s2), fun(fun) {}

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

//BinaryOperation operator+(Signal& s1, double s2) {
//    return BinaryOperation(s1, constant(s2), std::plus<double>());
//}

//BinaryOperation operator+(double s1, Signal& s2) {
//    BinaryOperation(constant(s1), s2, std::plus<double>());
//}


}
