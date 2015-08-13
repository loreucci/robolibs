#include "experimentlogger.h"

#include <iostream>
#include <csignal>
#include <chrono>
#include <ctime>

void handler(int) {

    explogger.signalHandler();
    exit(0);

}

ExperimentLogger explogger = ExperimentLogger();

ExperimentLogger::ExperimentLogger() {

    // TODO
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    struct tm* timeinfo = localtime(&now_c);
    // std::string time = std::string(std::put_time(&now_c, "%c %Z")); not yet supported
    char buffer[80];
    std::strftime(buffer,80,"%F-%H%M%S",timeinfo);
    timestamp = buffer;
    basename = "test";
    extension = ".txt";

    separator = " ";

    counter = 0;

    disabled = false;

    std::signal(SIGINT, handler);

}

ExperimentLogger::~ExperimentLogger() {
    deleteall();
}

void ExperimentLogger::setBasename(const std::string& basename) {
    this->basename = basename;
}

void ExperimentLogger::setExtension(const std::string& extension) {
    this->extension = extension;
}

std::string ExperimentLogger::getFilename() {
    return basename+timestamp;
}

void ExperimentLogger::setSeparator(const std::string& separator) {
    this->separator = separator;
}

void ExperimentLogger::reset() {
    deleteall();
}

void ExperimentLogger::addListener(Listener* vl) {

    listeners.push_back(vl);

}

void ExperimentLogger::readValues() {

    if (disabled)
        return;

    std::vector<double> tmp;
    for (Listener* l : listeners) {
        l->read();
    }
    counter++;

}

void ExperimentLogger::signalHandler() {
    if (disabled)
        return;
    saveToFile();
    std::cout << "Results saved to file." << std::endl;
    exit(0);
}

void ExperimentLogger::saveToFile() {
    if (disabled)
        return;
    std::ofstream file(basename+timestamp+extension);
    toFile(file);
    file.close();
}

void ExperimentLogger::disableLogging() {
    disabled = true;
}

bool ExperimentLogger::isDisabled() {
    return disabled;
}

void ExperimentLogger::printResults() {
    toFile(std::cout);
}

void ExperimentLogger::toFile(std::ostream& o) {

    if (disabled)
        return;

    for (auto l : listeners) {
        o << l->getName() << separator;
    }
    o << std::endl;

    for (unsigned int i = 0; i < counter; i++) {
        for (auto l : listeners) {
            o << l->getLine(i, separator);
        }
        o << std::endl;
    }

}

void ExperimentLogger::deleteall() {
    for (Listener* l : listeners)
        delete l;
    listeners.clear();
}
