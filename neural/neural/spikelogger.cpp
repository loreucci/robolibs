#include "spikelogger.h"

#include <fstream>
#include <iostream>

#include <sec/commons.h>


namespace neural {


SpikeLogger::SpikeLogger(const std::string& filename)
    :Node(0.0), filename(filename) {

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
        std::cerr << "DataLogger: unable to create spike logfile " << prefix + filename << std::endl;
        return false;
    }

    file << "ID time" << std::endl;
    for (const Spike& sp : data) {
        file << sp.neuron_id << " " << sp.time << std::endl;
    }

    file.close();
    return true;

}


}

void sec::connect(neural::SpikeNodeOut* out, neural::SpikeLogger& sink) {
    sink.addSpikeSource(out);
}
