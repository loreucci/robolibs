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

    void changeValue(const T& newvalue) {
        value = newvalue;
    }

    NodeOut<T> output;

protected:
    T value;

};

class SignalSource : public Source {

public:
    SignalSource(Signals::Signal signal, double samplingfreq = 0.0);

    virtual void execute() override;

    virtual std::string parameters() const override;

    virtual void reset() override;

    NodeOut<double> output;

protected:
    Signals::Signal signal;

};

class SignalSourceVector : public Source {

public:
    SignalSourceVector(const std::vector<Signals::Signal>& signalvec, double samplingfreq = 0.0);

    virtual void execute() override;

    virtual std::string parameters() const override;

    virtual void reset() override;

    NodeOut<Utils::Vector> output;

protected:
    std::vector<Signals::Signal> signalvec;

};


class FileSource : public Source {

public:
    FileSource(const std::string& filename, unsigned int ignorelines = 0, bool repeat = false, double samplingfreq = 0.0);

    virtual void execute() override;

    virtual std::string parameters() const override;

    virtual void reset() override;

    bool valid() const;

    NodeOut<std::vector<double>> output;

protected:
    std::string filename;
    unsigned int ignorelines;
    bool repeat, over;
    std::ifstream file;

    void skiplines();

};

class AbsoluteClock : public Source {

public:
    // Format must be specified according to std::strftime specifications.
    // See http://en.cppreference.com/w/cpp/chrono/c/strftime
    AbsoluteClock(const std::string& format, double freq = 100.0);

    virtual void execute() override;

    virtual std::string parameters() const override;

    NodeOut<double> epoch;
    NodeOut<std::string> formatted;
    NodeOut<std::chrono::time_point<std::chrono::system_clock>> clock;

private:
    std::string format;

};

}


#endif // SIMPLESOURCES_H
