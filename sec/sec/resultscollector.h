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

#ifndef RESULTSCOLLECTOR_H
#define RESULTSCOLLECTOR_H

#include <string>
#include <vector>

#include "node.h"

namespace sec {

enum Mode {
    FOLDER_MODE = 0,
    SINGLE_FILES_MODE = 1,
    FOLDER_MODE_TRIALS = 2,
    SINGLE_FILES_MODE_TRIALS = 3,
};

class Logger {

public:
    virtual bool logToFile() const = 0;
    virtual void setPrefix(const std::string& prefix) = 0;

};

const std::string logfilename = "seclogs.txt";

class ResultsCollector {

public:

    ResultsCollector(const std::string& basename = "test", Mode mode = FOLDER_MODE);

    // this should be called by the logger with the filename without the prefix
    void registerLogger(Logger* logger, const std::string& filename);
    void registerExtraFile(const std::string& filename);

    // this should return:
    // "basename-timestamp/" if FOLDER_MODE
    // "basename-timestamp_" if SINGLE_FILE_MODE
    // so that the loggers can just append
    std::string getFilenamePrefix();

    void setBasename(const std::string& basename);
    void setMode(Mode mode);

    void saveNodesParameters(std::vector<Node*> nodes, const std::string& filename = "parameters.txt");
    void saveAllNodesParameters(const std::string& filename = "parameters.txt");

    void toggleLogging(bool toggle = false);

    void saveAll();

    // it can be used if external results need to be saved before the end of the experiment
    bool createFolder();

    // multiple trials handling
    void setTrials(unsigned int numtrials);
    void setCurrentTrial(unsigned int currentTrial);

protected:
    std::string basename, timestamp;
    Mode mode;
    std::vector<std::pair<Logger*, std::string>> loggers;
    std::vector<std::string> extrafiles;
    bool saved, enabled;

    bool folder_created;

    void createExplorerEntry();

    // trials
    unsigned int padding, currentTrial;
    std::string paddedTrial();

};

extern ResultsCollector results_collector;

// convenience functions
void setResultsName(const std::string& basename);
void setResultsMode(Mode mode);
void saveAllNodesParameters(const std::string& filename = "parameters.txt");

// in order to have extra files tracked by the system
// one should use these functions and then create the file
// as prefix+filename
std::string getResultsPrefix();
void registerResultsFile(const std::string& filename);

}

#endif // RESULTSCOLLECTOR_H
