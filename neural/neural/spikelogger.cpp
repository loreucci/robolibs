#include "spikelogger.h"

#include <fstream>

#include <sec/commons.h>

namespace neural {


SpikeLogger::SpikeLogger(const std::string& basename, const std::string& extension)
    :Node(0.0), basename(basename), extension(extension) {

    listeners.clear();
    timestamp = sec::getUniqueTimeStamp();

}

SpikeLogger::~SpikeLogger() {

    saveToFile();

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
    return "Spike logger.";
}

void SpikeLogger::addSpikeSource(SpikeNodeOut* source) {

    SpikeNodeIn* in = new SpikeNodeIn(source);
    source->addConnection(in);
    listeners.push_back(in);

}

void SpikeLogger::saveToFile() const {

    if (!enabled)
        return;

    std::ofstream file(basename+timestamp+extension);
    file << "ID time" << std::endl;
    for (const Spike& sp : data) {
        file << sp.neuron_id << " " << sp.time << std::endl;
    }

    file.close();

}

void SpikeLogger::toggleLogging(bool toggle) {
    enabled = toggle;
}


}

void sec::connect(neural::SpikeNodeOut* out, neural::SpikeLogger& sink) {
    sink.addSpikeSource(out);
}
