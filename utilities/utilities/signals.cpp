#include "signals.h"

#include <random>

#include <cmath>
#include "utilities.h"

namespace Signals {


Signal::Signal(std::function<double(double)> fun, const std::string& description, double samplingfreq)
    :fun(fun), description(description) {

    setSamplingFreq(samplingfreq);

    t = 0;

}

double Signal::operator()() {
    return output();
}

double Signal::output() {
    return fun(t++/samplingfreq);
}

void Signal::reset(double time) {

    t = time * samplingfreq;

}

std::string Signal::to_string() const {
    return description;
}

std::function<double(double)> Signal::getFunction() const {
    return fun;
}

double Signal::getSamplingFreq() const {
    return samplingfreq;
}

void Signal::setSamplingFreq(double samplingfreq) {
    if (samplingfreq < 0.0)
        throw std::invalid_argument("[Signal] Sampling frequency must be non-negative.");
    this->samplingfreq = samplingfreq;
}


Signal constant(double c, double samplingfreq) {
    return Signal([c](double) {return c;},
                  "[constant: " + std::to_string(c) + "]",
    samplingfreq);
}


Signal sin(double ampl, double freq, double phase, double samplingfreq) {

    std::string str = "[sin: ";
    str += "a=" + std::to_string(ampl) + ", ";
    str += "f=" + std::to_string(freq) + ", ";
    str += "ph=" + std::to_string(phase) + "]";

    auto fun = [ampl, freq, phase] (double t) {
        return ampl * std::sin(2*Utils::PI*freq*t + phase);
    };

    return Signal(fun, str, samplingfreq);

}


Signal ramp(double slope, double initialvalue, double starttime, double samplingfreq) {

    std::string str = "[ramp: ";
    str += "slope=" + std::to_string(slope) + ", ";
    str += "initialvalue=" + std::to_string(initialvalue) + ", ";
    str += "starttime=" + std::to_string(starttime) + "]";

    auto fun = [slope, initialvalue, starttime] (double t) {
        if (t <= starttime) {
            return initialvalue;
        }
        return slope*(t-starttime)+initialvalue;
    };

    return Signal(fun, str, samplingfreq);

}

Signal rampandhold(double slope, double initialvalue, double stoptime, double starttime, double samplingfreq) {

    std::string str = "[rampandhold: ";
    str += "slope=" + std::to_string(slope) + ", ";
    str += "initialvalue=" + std::to_string(initialvalue) + ", ";
    str += "stoptime=" + std::to_string(stoptime) + ", ";
    str += "starttime=" + std::to_string(starttime) + "]";

    auto fun = [slope, initialvalue, stoptime, starttime] (double t) {
        static double lastvalue = initialvalue;
        if (t <= starttime)
            lastvalue = initialvalue;
        else if (t <= stoptime)
            lastvalue = slope*(t-starttime)+initialvalue;
        return lastvalue;
    };

    return Signal(fun, str, samplingfreq);

}

Signal chirp(double ampl, double f0, double k, double phase, double samplingfreq) {

    std::string str = "[chirp: ";
    str += "a=" + std::to_string(ampl) + ", ";
    str += "f0=" + std::to_string(f0) + ", ";
    str += "k=" + std::to_string(k) + ", ";
    str += "phase=" + std::to_string(phase) + "]";

    auto fun = [ampl, f0, k, phase] (double t) {
        return ampl * std::sin(2*Utils::PI*(f0*t + k/2*std::pow(t, 2)) + phase);
    };


    return Signal(fun, str, samplingfreq);

}

Signal noise(double mean, double stddev, double samplingfreq) {

    std::random_device rd;
    std::mt19937 gen(rd());

    // values near the mean are the most likely
    // standard deviation affects the dispersion of generated values from the mean
    std::normal_distribution<> d(mean, stddev);

    std::string str = "[noise: ";
    str += "mean=" + std::to_string(mean) + ", ";
    str += "stddev=" + std::to_string(stddev) + "]";

    auto fun = [d, gen] (double) mutable {
        return d(gen);
    };

    return Signal(fun, str, samplingfreq);

}


Signal Switch(Signal s1, Signal s2, double switchtime, bool shift, double samplingfreq) {

    if (samplingfreq == 0.0) {
        samplingfreq = s1.getSamplingFreq();
    }
    if (samplingfreq == 0.0) {
        samplingfreq = s2.getSamplingFreq();
    }

    std::string str = "[switch: s1=" + s1.to_string() + ", s2=" + s2.to_string() + ", time=" + std::to_string(switchtime) + "]";

    auto f1 = s1.getFunction();
    auto f2 = s2.getFunction();
    auto fun = [switchtime, f1, f2, shift] (double t) {
        double out1 = f1(t);
        double out2;
        if (!shift)
            out2 = f2(t);

        if (t <= switchtime) {
            return out1;
        }

        if (shift)
            out2 = f2(t-switchtime);
        return out2;
    };

    return Signal(fun, str, samplingfreq);

}

Signal BinaryOperation(Signal s1, Signal s2, std::function<double (double, double)> op, double samplingfreq) {

    if (samplingfreq == 0.0) {
        samplingfreq = s1.getSamplingFreq();
    }
    if (samplingfreq == 0.0) {
        samplingfreq = s2.getSamplingFreq();
    }

    std::string str = "[operation: s1=" + s1.to_string() + ", s2=" + s2.to_string() + "]";

    auto f1 = s1.getFunction();
    auto f2 = s2.getFunction();
    auto fun = [op, f1, f2] (double t) {
        return op(f1(t), f2(t));
    };

    return Signal(fun, str, samplingfreq);

}

Signal operator+(Signal s1, Signal s2) {
    return BinaryOperation(s1, s2, std::plus<double>());
}

Signal operator-(Signal s1, Signal s2) {
    return BinaryOperation(s1, s2, std::minus<double>());
}

Signal operator*(Signal s1, Signal s2) {
    return BinaryOperation(s1, s2, std::multiplies<double>());
}

Signal operator/(Signal s1, Signal s2) {
    return BinaryOperation(s1, s2, std::divides<double>());
}


Signal operator+(double c, Signal s) {
    return BinaryOperation(constant(c), s, std::plus<double>(), s.getSamplingFreq());
}

Signal operator+(Signal s, double c) {
    return BinaryOperation(s, constant(c), std::plus<double>(), s.getSamplingFreq());
}

Signal operator-(double c, Signal s) {
    return BinaryOperation(constant(c), s, std::minus<double>(), s.getSamplingFreq());
}

Signal operator-(Signal s, double c) {
    return BinaryOperation(s, constant(c), std::minus<double>(), s.getSamplingFreq());
}

Signal operator*(double c, Signal s) {
    return BinaryOperation(constant(c), s, std::multiplies<double>(), s.getSamplingFreq());
}

Signal operator*(Signal s, double c) {
    return BinaryOperation(s, constant(c), std::multiplies<double>(), s.getSamplingFreq());
}

Signal operator/(double c, Signal s) {
    return BinaryOperation(constant(c), s, std::divides<double>(), s.getSamplingFreq());
}

Signal operator/(Signal s, double c) {
    return BinaryOperation(s, constant(c), std::divides<double>(), s.getSamplingFreq());
}

}
