#include "logger.h"

#include <fstream>
#include <iostream>
#include <chrono>


namespace sec {

Logger::Logger(const std::string& basename, const std::string& separator, const std::string& extension)
    :Node(0.0), basename(basename), separator(separator), extension(extension) {

    enabled = true;
    counter = 0;

    // TODO
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    struct tm* timeinfo = localtime(&now_c);
    // std::string time = std::string(std::put_time(&now_c, "%c %Z")); not yet supported
    char buffer[80];
    std::strftime(buffer,80,"%F-%H%M%S",timeinfo);
    timestamp = buffer;

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
}

void Logger::toFile(std::ostream& o) const {

    if (!enabled)
        return;

    o << "Phase<" << counter << ">" << separator;
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
