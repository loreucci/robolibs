#include "signals.h"

#include <cmath>
#include "utilities.h"

namespace Signals {


Signal::Signal(std::function<double (unsigned int, double)> fun, const std::string& description, double samplingfreq)
    :fun(fun), description(description) {

    setSamplingFreq(samplingfreq);

    t = 0;

}

double Signal::operator()() {
    return output();
}

double Signal::output() {
    return fun(t++, samplingfreq);
}

void Signal::reset(double time) {

    t = time * samplingfreq;

}

std::string Signal::to_string() const {
    return description;
}

std::function<double(unsigned int, double)> Signal::getFunction() const {
    return fun;
}

double Signal::getSamplingFreq() const {
    return samplingfreq;
}

void Signal::setSamplingFreq(double samplingfreq) {
    if (samplingfreq < 0.0)
        throw std::invalid_argument("Signal: sampling frequency must be non-negative.");
    this->samplingfreq = samplingfreq;
}


Signal constant(double c, double samplingfreq) {
    return Signal([c](unsigned int, double) {return c;},
                  "[constant: " + std::to_string(c) + "]",
    samplingfreq);
}


Signal sin(double ampl, double freq, double phase, double samplingfreq) {

    std::string str = "[sin: ";
    str += "a=" + std::to_string(ampl) + ", ";
    str += "f=" + std::to_string(freq) + ", ";
    str += "ph=" + std::to_string(phase) + "]";

    auto fun = [ampl, freq, phase] (unsigned int t, double samplingfreq) {
        return ampl * std::sin(2*Utils::PI*freq*t/samplingfreq + phase);
    };

    return Signal(fun, str, samplingfreq);

}


Signal ramp(double slope, double initialvalue, double starttime, double samplingfreq) {

    std::string str = "[ramp: ";
    str += "slope=" + std::to_string(slope) + ", ";
    str += "initialvalue=" + std::to_string(initialvalue) + ", ";
    str += "starttime=" + std::to_string(starttime) + "]";

    auto fun = [slope, initialvalue, starttime] (unsigned int t, double samplingfreq) {
        double time = t / samplingfreq;
        if (time <= starttime) {
            return initialvalue;
        }
        return slope*(time-starttime);
    };

    return Signal(fun, str, samplingfreq);

}

Signal rampandhold(double slope, double initialvalue, double stoptime, double starttime, double samplingfreq) {

    std::string str = "[rampandhold: ";
    str += "slope=" + std::to_string(slope) + ", ";
    str += "initialvalue=" + std::to_string(initialvalue) + ", ";
    str += "stoptime=" + std::to_string(stoptime) + ", ";
    str += "starttime=" + std::to_string(starttime) + "]";

    auto fun = [slope, initialvalue, stoptime, starttime] (unsigned int t, double samplingfreq) {
        double time = t / samplingfreq;
        static double lastvalue = initialvalue;
        if (time <= starttime)
            lastvalue = initialvalue;
        else if (time <= stoptime)
            lastvalue = slope*(time-starttime);
        return lastvalue;
    };

    return Signal(fun, str, samplingfreq);

}


Signal Switch(Signal s1, Signal s2, double switchtime, double samplingfreq) {

    if (samplingfreq == 0.0) {
        samplingfreq = s1.getSamplingFreq();
    }
    if (samplingfreq == 0.0) {
        samplingfreq = s2.getSamplingFreq();
    }

    std::string str = "[switch: s1=" + s1.to_string() + ", s2=" + s2.to_string() + ", time=" + std::to_string(switchtime) + "]";

    auto f1 = s1.getFunction();
    auto f2 = s2.getFunction();
    auto fun = [switchtime, f1, f2] (unsigned int t, double samplingfreq) {
        double out1 = f1(t, samplingfreq);
        double out2 = f2(t, samplingfreq);

        if (t/samplingfreq <= switchtime) {
            return out1;
        }
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
    auto fun = [op, f1, f2] (unsigned int t, double samplingfreq) {
        return op(f1(t, samplingfreq), f2(t, samplingfreq));
    };

    return Signal(fun, str, samplingfreq);

}

Signal operator+(Signal s1, Signal s2) {
    return BinaryOperation(s1, s2, std::plus<double>());
}

Signal operator-(Signal& s1, Signal& s2) {
    return BinaryOperation(s1, s2, std::minus<double>());
}

Signal operator*(Signal& s1, Signal& s2) {
    return BinaryOperation(s1, s2, std::multiplies<double>());
}

Signal operator/(Signal& s1, Signal& s2) {
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
