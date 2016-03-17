#include "simplesources.h"

#include <cmath>
#include <sstream>

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


FileSource::FileSource(const std::string& filename, unsigned int ignorelines, bool repeat, double samplingfreq)
    :Source(samplingfreq), filename(filename), ignorelines(ignorelines), repeat(repeat) {

    file.open(filename);
    if (!file.good())
        throw std::runtime_error("FileSource: unable to open file " + filename);

    skiplines();

    over = false;

}

void FileSource::execute() {

    if (over)
        return;

    std::string str;
    std::getline(file, str);

    std::stringstream stream(str);
    std::vector<double> out;
    while (stream.good()) {
        double d;
        stream >> d;
        out.push_back(d);
    }

    output = out;

    // check if file is over
    char c;
    file >> c;
    if (!file.eof()) {
        file.putback(c);
    } else {
        if (repeat) {
            file.clear();
            file.seekg(0);
            skiplines();
        } else {
            over = true;
        }
    }

}

std::string FileSource::parameters() const {
    return "FilesSource with file: " + filename;
}

bool FileSource::valid() const {
    return repeat || !over;
}

void FileSource::skiplines() {
    std::string str;
    for (unsigned int i = 0; i < ignorelines; i++) {
        std::getline(file, str);
    }
}


}
