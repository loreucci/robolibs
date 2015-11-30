#include "simplesources.h"

#include <cmath>

#include <utilities/utilities.h>

namespace sec {

SinusoidalSource::SinusoidalSource(double ampl, double freq, double phase, double mean, double samplingfreq)
    :Source(samplingfreq), ampl(ampl), freq(freq), phase(phase), mean(mean) {

    step = 0;

}

void SinusoidalSource::execute() {
    double ret = ampl * std::sin(2*Utils::PI*freq*step/getFrequency() + phase) + mean;
    step++;
    output.addData(ret);
}

std::string SinusoidalSource::parameters() const {
    std::string ret = "Sinusoidal source:\n";
    ret += "\tampl = " + std::to_string(ampl) + "\n";
    ret += "\tfreq = " + std::to_string(freq) + "\n";
    ret += "\tphase = " + std::to_string(phase);
    return ret;
}


}
