#ifndef RESULTSCOLLECTOR_H
#define RESULTSCOLLECTOR_H

#include <string>
#include <vector>

#include "node.h"

namespace sec {

enum Mode {
    FOLDER_MODE = 0,
    SINGLE_FILES_MODE
};

class Logger {

public:
    virtual bool logToFile() const = 0;

};

class ResultsCollector {

public:

    ResultsCollector(const std::string& basename = "test", Mode mode = FOLDER_MODE);
//    ~ResultsCollector();

    // this should be called by the logger with the filename without the prefix
    void registerLogger(const Logger* logger, const std::string& filename);
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

protected:
    std::string basename, timestamp;
    Mode mode;
    std::vector<std::pair<const Logger*, std::string>> loggers;
    std::vector<std::string> extrafiles;
    bool saved, enabled;

    bool folder_created;

    void createExplorerEntry();

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
