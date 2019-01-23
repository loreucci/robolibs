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

#include "resultscollector.h"

#include <stdexcept>
#include <fstream>
#include <iostream>
#include <cstring>
#include <cmath>

#include <sys/stat.h>

#include "commons.h"
#include "controller.h"
#include "flags.h"


namespace sec {


ResultsCollector::ResultsCollector(const std::string &basename, Mode mode)
    :basename(basename), mode(mode) {

    saved = false;
    enabled = true;
    folder_created = false;

    timestamp = getUniqueTimeStamp();

}

void ResultsCollector::registerLogger(Logger* logger, const std::string &filename) {

    loggers.push_back(std::make_pair(logger, filename));
}

void ResultsCollector::registerExtraFile(const std::string &filename) {
    extrafiles.push_back(filename);
}

std::string ResultsCollector::getFilenamePrefix() {

    switch (mode) {
    case FOLDER_MODE:
        return basename+"-"+timestamp+"/";
    case SINGLE_FILES_MODE:
        return basename+"-"+timestamp+"_";
    case FOLDER_MODE_TRIALS:
        return basename+"-"+timestamp+"/trial_"+paddedTrial()+"/";
    case SINGLE_FILES_MODE_TRIALS:
        return basename+"-"+timestamp+"/trial_"+paddedTrial()+"_";
    default:
        throw std::runtime_error("[ResultsCollector] Unkown mode.");
    }

}

void ResultsCollector::setBasename(const std::string &basename) {
    this->basename = basename;
}

void ResultsCollector::setMode(Mode mode) {
    this->mode = mode;
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
    bool ok = createFolder();

    // save files
    for (auto l : loggers) {
        ok = l.first->logToFile() && ok;
    }

    // log results for ResultsExplorer
    createExplorerEntry();

    saved = true;

    if (ok) {
        if (sec::isVerbose())
            std::cerr << "[ResultsCollector] Results saved to file." << std::endl;
    } else
        std::cerr << "[ResultsCollector] Error logging some files." << std::endl;

}

bool ResultsCollector::createFolder() {

    if (folder_created)
        return true;

    if (mode == FOLDER_MODE) {

        // waiting for filesystem library to make this code standard...
        // drwxr-xr-x
        if (mkdir((basename+"-"+timestamp).c_str(), 0755) == -1) {
            std::cerr << "[ResultsCollector] Error while creating the folder. " << std::strerror(errno) << std::endl;
            return false;
        }

    }

    if (mode == FOLDER_MODE_TRIALS) {

        if (mkdir((basename+"-"+timestamp+"/trial_"+paddedTrial()).c_str(), 0755) == -1) {
            std::cerr << "[ResultsCollector] Error while creating the folder. " << std::strerror(errno) << std::endl;
            return false;
        }

    }

    folder_created = true;
    return true;
}

void ResultsCollector::setTrials(unsigned int numtrials) {

    // padding for filenames
    padding = std::floor(std::log10(numtrials));

    // change mode
    if (mode == FOLDER_MODE)
        mode = FOLDER_MODE_TRIALS;
    if (mode == SINGLE_FILES_MODE)
        mode = SINGLE_FILES_MODE_TRIALS;

    if (loggers.empty() && extrafiles.empty())
        return;

    // create folder for trials
    if (mkdir((basename+"-"+timestamp).c_str(), 0755) == -1) {
        std::cerr << "[ResultsCollector] Error while creating the trials folder. " << std::strerror(errno) << std::endl;
    }


}

void ResultsCollector::setCurrentTrial(unsigned int currentTrial) {

    this->currentTrial = currentTrial;

    for (auto& l : loggers) {
        l.first->setPrefix(getFilenamePrefix());
    }

    if (mode == FOLDER_MODE_TRIALS)
        folder_created = false;

    saved = false;

}

void ResultsCollector::createExplorerEntry() {

    std::ofstream logs(logfilename, std::ios_base::app);
    logs << mode << ";" << basename << ";" << timestamp << ";";
    if (mode == FOLDER_MODE_TRIALS || mode == SINGLE_FILES_MODE_TRIALS)
        logs << paddedTrial() << ";";
    else
        logs << "-;";
    std::string names = "";
    for (auto l : loggers)
        names += l.second + ",";
    for (auto ef : extrafiles)
        names += ef + ",";
    names.resize(names.length()-1);
    logs << names;
    logs << ";;"; // empty comment and exportname
    logs << "\n";
    logs.close();

}

std::string ResultsCollector::paddedTrial() {

    unsigned int len = std::floor(std::log10(currentTrial));

    std::string ret = "";
    for (unsigned int i = 0; i < padding - len; i++)
        ret += "0";

    return ret + std::to_string(currentTrial);

}


ResultsCollector results_collector;

void setResultsName(const std::string &basename) {
    results_collector.setBasename(basename);
}

void setResultsMode(Mode mode) {
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
