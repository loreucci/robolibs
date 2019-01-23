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

#include "spikelogger.h"

#include <fstream>
#include <iostream>

#include <sec/commons.h>


namespace neural {


SpikeLogger::SpikeLogger(const std::string& filename, double freq)
    :Node(freq), filename(filename) {

    listeners.clear();

    prefix = sec::results_collector.getFilenamePrefix();
    sec::results_collector.registerLogger(this, filename);

}

SpikeLogger::~SpikeLogger() {

    for (auto& l : listeners) {
        delete l;
    }

}

void SpikeLogger::refreshInputs() {

    for (auto& l : listeners)
        l->refreshData();

}

bool SpikeLogger::connected() const {

    for (auto& l : listeners)
        if (!l->isConnected())
            return false;

    return true;

}

void SpikeLogger::execute() {

    for (auto& l : listeners)
        data.append(l->getData());

}

std::string SpikeLogger::parameters() const {
    return "Spike logger (" + filename + ").";
}

void SpikeLogger::addSpikeSource(SpikeNodeOut* source) {

    SpikeNodeIn* in = new SpikeNodeIn(source);
    source->addConnection(in);
    listeners.push_back(in);

}

bool SpikeLogger::logToFile() const {

    std::ofstream file(prefix+filename);
    if (!file.good()) {
        std::cerr << "[SpikeLogger] unable to create spike logfile " << prefix + filename << std::endl;
        return false;
    }

    file << "ID time" << std::endl;
    for (const Spike& sp : data) {
        file << sp.neuron_id << " " << sp.time << std::endl;
    }

    file.close();
    return true;

}

void SpikeLogger::reset() {
    data.clear();
}

void SpikeLogger::setPrefix(const std::string& prefix) {
    this->prefix = prefix;
}


}

void sec::connect(neural::SpikeNodeOut& out, neural::SpikeLogger& sink) {
    sink.addSpikeSource(&out);
}
