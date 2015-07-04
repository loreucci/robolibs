#ifndef EXPERIMENTLOGGER_H
#define EXPERIMENTLOGGER_H

#include <string>
#include <ostream>
#include <functional>
#include <fstream>

#include "listener.h"
#include "source.h"
#include "controller.h"

class ExperimentLogger {

public:
    ExperimentLogger();

    ~ExperimentLogger();

    void setBasename(const std::string& basename);
    void setExtension(const std::string& extension);
    void setSeparator(const std::string& separator);
    std::string getFilename();

    void reset();

    void addListener(Listener* vl);
    template <typename T>
    void addListener(T& t) {
        listeners.push_back(new ValueListener<T>(t));
    }
    template <typename T>
    void addListener(std::function<T(void)> fn) {
        listeners.push_back(new FunctionListener<T>(fn));
    }
    template <typename C, typename T>
    void addListener(const C& c, std::function<T(const C&)> fn) {
        listeners.push_back(new FunctionListener<T>([&c, fn](){return fn(c);}));
    }
    template <typename T, unsigned int S, unsigned int ID>
    void addListener(const Message<T, S, ID>& m) {
        listeners.push_back(new MessageListener<T, S, ID>(m));
    }

    template <typename T>
    void logParameters(const Source<T>& s) {
        if (disabled)
            return;
        std::ofstream file(basename+timestamp+"_parameters"+extension, std::ios_base::out | std::ios_base::app);
        file << s.parameters() << std::endl;
        file.close();
    }

    template <typename A, typename B>
    void logParameters(const Controller<A, B>& c) {
        if (disabled)
            return;
        std::ofstream file(basename+timestamp+"_parameters"+extension, std::ios_base::out | std::ios_base::app);
        file << c.parameters() << std::endl;
        file.close();
    }

    void readValues();

    void signalHandler();

    void saveToFile();

    void disableLogging();
    bool isDisabled();

    // debug only
    void printResults();


protected:
    std::string timestamp, basename, extension, separator;
    std::vector<Listener*> listeners;
    unsigned int counter;
//    std::vector<std::vector<double>> results;
    bool disabled;

    void toFile(std::ostream& o);

    void deleteall();

};

extern ExperimentLogger explogger;


#endif // EXPERIMENTLOGGER_H
