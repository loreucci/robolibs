#include "datalogger.h"

#include <fstream>
#include <chrono>

#include "commons.h"

namespace sec {

DataLogger::DataLogger(const std::string& separator, const std::string& filename)
    :Node(0.0), filename(filename), separator(separator) {

    counter = 0;

    prefix = results_collector.getFilenamePrefix();

    results_collector.registerLogger(this, filename);

}

DataLogger::~DataLogger() {
    deleteall();
}

void DataLogger::refreshInputs() {}

bool DataLogger::connected() const {
    return true;
}

void DataLogger::execute() {

    for (DataListener* l : listeners) {
        l->read();
    }
    counter++;

}

std::string DataLogger::parameters() const {
    return "Data logger (" + filename + ").";
}

void DataLogger::addListener(DataListener* l) {
    listeners.push_back(l);
}

void DataLogger::setFilename(const std::string& filename) {
    this->filename = filename;
}

void DataLogger::setSeparator(const std::string& separator) {
    this->separator = separator;
}

void DataLogger::reset() {
    deleteall();
}

void DataLogger::logToFile() const {

    std::ofstream file(prefix + filename);
    toFile(file);
    file.close();

}

void DataLogger::toFile(std::ostream& o) const {

    o << "T " << separator;
    for (auto l : listeners) {
        o << l->getName() << separator;
    }
    o << std::endl;

    for (unsigned int i = 0; i < counter; i++) {
        o << i / getFrequency() << separator;
        for (auto l : listeners) {
            o << l->getLine(i, separator) << separator;
        }
        o << std::endl;
    }

}

void DataLogger::deleteall() {
    for (DataListener* l : listeners)
        delete l;
    listeners.clear();
}

}
