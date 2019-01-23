/*
 * Copyright (C) 2014-2019 Lorenzo Vannucci
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

#include "simplesources.h"

#include <cmath>
#include <sstream>

#include <utilities/utilities.h>

namespace sec {


SignalSource::SignalSource(Signals::Signal signal, double samplingfreq)
    :Source(samplingfreq), signal(signal) {
    signal.setSamplingFreq(samplingfreq);
}

void SignalSource::execute() {
    output = signal();
}

std::string SignalSource::parameters() const {
    return "SignalSource with " + signal.to_string();
}

void SignalSource::reset() {
    signal.reset();
}


SignalSourceVector::SignalSourceVector(const std::vector<Signals::Signal>& signalvec, double samplingfreq)
    :Source(samplingfreq), signalvec(signalvec) {

    if (signalvec.empty())
        throw std::invalid_argument("[SignalSourceVector] Signalvector must not be empty.");

    output = Utils::Vector(signalvec.size());

}

void SignalSourceVector::execute() {

    Utils::Vector out(signalvec.size());
    for (unsigned int i = 0; i < signalvec.size(); i++) {
        out[i] = signalvec[i].output();
    }

    output = out;

}

std::string SignalSourceVector::parameters() const {

    std::string ret = "SignalSourceVector with [";
    for (unsigned int i = 0; i < signalvec.size(); i++) {
        ret += "\n\t" + signalvec[i].to_string();
    }
    ret += "\n\t]";

    return ret;
}

void SignalSourceVector::reset() {

    for (unsigned int i = 0; i < signalvec.size(); i++) {
        signalvec[i].reset();
    }

}


FileSource::FileSource(const std::string& filename, unsigned int ignorelines, bool repeat, double samplingfreq)
    :Source(samplingfreq), filename(filename), ignorelines(ignorelines), repeat(repeat) {

    file.open(filename);
    if (!file.good())
        throw std::runtime_error("[FileSource] Unable to open file " + filename);

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
    return "FileSource with file: " + filename;
}

void FileSource::reset() {
    file.clear();
    file.seekg(0);
    skiplines();
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


AbsoluteClock::AbsoluteClock(const std::string& format, double freq)
    :sec::Source(freq), format(format) {}

void AbsoluteClock::execute() {

    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    struct tm* timeinfo = localtime(&now_c);
    // std::string time = std::string(std::put_time(&now_c, "%c %Z")); not yet supported
    char buffer[80];
    std::strftime(buffer,80,format.c_str(),timeinfo);
    formatted = buffer;
    epoch = std::chrono::duration_cast<std::chrono::duration<double>>(now.time_since_epoch()).count();
    clock = now;

}

std::string AbsoluteClock::parameters() const {
    return "Absolute clock (format = " + format + ")";
}


}
