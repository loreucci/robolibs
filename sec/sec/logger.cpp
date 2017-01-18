#include "logger.h"

#include <fstream>
#include <iostream>
#include <chrono>

#include "commons.h"

namespace sec {

Logger::Logger(const std::string& basename, const std::string& separator, const std::string& extension)
    :Node(0.0), basename(basename), separator(separator), extension(extension) {

    enabled = true;
    counter = 0;

    parameters_logged = false;

    timestamp = getUniqueTimeStamp();

}

Logger::~Logger() {
    saveToFile();
    deleteall();
}

void Logger::refreshInputs() {}

bool Logger::connected() const {
    return true;
}

void Logger::execute() {

    if (!enabled)
        return;

    for (Listener* l : listeners) {
        l->read();
    }
    counter++;

}

std::string Logger::parameters() const {
    return "Experiment logger.";
}

void Logger::addListener(Listener* l) {
    listeners.push_back(l);
}

void Logger::setBasename(const std::string& basename) {
    this->basename = basename;
}

void Logger::setExtension(const std::string& extension) {
    this->extension = extension;
}

void Logger::setSeparator(const std::string& separator) {
    this->separator = separator;
}

void Logger::reset() {
    deleteall();
}

void Logger::toggleLogging(bool toggle) {
    enabled = toggle;
}

void Logger::saveToFile() const {
    if (!enabled)
        return;
    std::ofstream file(basename+timestamp+extension);
    toFile(file);
    file.close();

    std::ofstream logs("logs.txt", std::ios_base::app);
    logs << basename+timestamp+extension << ";" << (parameters_logged ? 1 : 0) << ";;\n";
    logs.close();

}

void Logger::logNodes(std::vector<Node*> nodes) const {

    std::ofstream file(basename+timestamp+"_parameters"+extension, std::ios_base::out | std::ios_base::app);
    for (const auto n : nodes){
        file << n->parameters() << std::endl;
    }
    file.close();
    parameters_logged = true;

}

void Logger::toFile(std::ostream& o) const {

    if (!enabled)
        return;

    o << "T " << separator;
    for (auto l : listeners) {
        o << l->getName() << separator;
    }
    o << std::endl;

    for (unsigned int i = 0; i < counter; i++) {
        o << i / getFrequency() << separator;
//        o << i << separator;
        for (auto l : listeners) {
            o << l->getLine(i, separator) << separator;
        }
        o << std::endl;
    }

    std::cout << "Results saved to file." << std::endl;

}

void Logger::deleteall() {
    for (Listener* l : listeners)
        delete l;
    listeners.clear();
}

}
