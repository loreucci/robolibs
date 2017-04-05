#include "resultscollector.h"

#include <stdexcept>
#include <fstream>
#include <iostream>

#include <sys/stat.h>

#include "commons.h"
#include "controller.h"


namespace sec {


ResultsCollector::ResultsCollector(const std::string &basename, Mode mode)
    :basename(basename), mode(mode) {

    saved = false;
    enabled = true;
    folder_created = false;

    timestamp = getUniqueTimeStamp();

}

//ResultsCollector::~ResultsCollector() {
//    saveAll();
//}

void ResultsCollector::registerLogger(const Logger *logger, const std::string &filename) {
//    if (mode == SINGLE_FILE_MODE && loggers.size() > 0) {
//        throw std::runtime_error("ResultsCollector: trying to register more than one logger in SINGLE_FILE_MODE.");
//    }

    loggers.push_back(std::make_pair(logger, filename));
}

void ResultsCollector::registerExtraFile(const std::__cxx11::string &filename) {
    extrafiles.push_back(filename);
}

std::string ResultsCollector::getFilenamePrefix() {

    switch (mode) {
    case FOLDER_MODE:
        return basename+"-"+timestamp+"/";
    case SINGLE_FILES_MODE:
        return basename+"-"+timestamp+"_";
    default:
        throw std::runtime_error("ResultsCollector: unkown mode.");
    }

}

void ResultsCollector::setBasename(const std::string &basename) {
    this->basename = basename;
}

void ResultsCollector::setMode(ResultsCollector::Mode mode) {
    this->mode = mode;

//    if (mode == SINGLE_FILE_MODE && loggers.size() > 0) {
//        throw std::runtime_error("ResultsCollector: trying to register more than one logger in SINGLE_FILE_MODE.");
//    }
}

void ResultsCollector::saveNodesParameters(std::vector<Node *> nodes, const std::string &filename) {

    createFolder();

    std::ofstream file(getFilenamePrefix()+filename, std::ios_base::out | std::ios_base::app);
    for (const auto n : nodes) {
        file << n->parameters() << std::endl;
    }
    file.close();

    registerExtraFile(filename);

}

void ResultsCollector::saveAllNodesParameters(const std::string &filename) {
    saveNodesParameters(main_controller.getAllNodes(), filename);
}

void ResultsCollector::toggleLogging(bool toggle) {
    enabled = toggle;
}

void ResultsCollector::saveAll() {

    if (saved)
        return;

    if (loggers.empty() && extrafiles.empty())
        return;

    // create folder if needed
    if (mode == FOLDER_MODE) {

        // waiting for filesystem library to make this code standard...
        // drwxr-xr-x
        mkdir((basename+"-"+timestamp).c_str(), 0755);

    }

    // save files
    for (auto l : loggers) {
        l.first->logToFile();
    }

    // log results for ResultsExplorer
    createExplorerEntry();

    saved = true;

    std::cout << "Results saved to file." << std::endl;

}

void ResultsCollector::createFolder() {

    if (folder_created)
        return;

    if (mode == FOLDER_MODE) {

        // waiting for filesystem library to make this code standard...
        // drwxr-xr-x
        mkdir((basename+"-"+timestamp).c_str(), 0755);

    }

    folder_created = true;
}

void ResultsCollector::createExplorerEntry() {

    std::ofstream logs("logs.txt", std::ios_base::app);
    logs << mode << ";" << basename << ";" << timestamp << ";";
    std::string names = "";
    for (auto l : loggers)
        names += l.second + ",";
    for (auto ef : extrafiles)
        names += ef + ",";
    names.resize(names.length()-1);
    logs << names << ";;\n";
    logs.close();

}


ResultsCollector results_collector;

void setResultsName(const std::string &basename) {
    results_collector.setBasename(basename);
}

void setResultsMode(ResultsCollector::Mode mode) {
    results_collector.setMode(mode);
}

void saveAllNodesParameters(const std::string& filename) {
    results_collector.saveAllNodesParameters(filename);
}

std::string getResultsPrefix() {
    return results_collector.getFilenamePrefix();
}

void registerResultsFile(const std::string& filename) {
    results_collector.registerExtraFile(filename);
}


}
