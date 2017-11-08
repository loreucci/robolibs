#ifndef DATALOGGER_H
#define DATALOGGER_H

#include <string>

#include "node.h"
#include "datalistener.h"
#include "resultscollector.h"


namespace sec {

class DataLogger : public Node, public Logger {

public:
    DataLogger(const std::string& separator = " ", const std::string& filename = "data.txt", double freq = 0.0);

    virtual ~DataLogger();

    virtual void refreshInputs() override;

    virtual bool connected() const override;

    virtual void execute() override;

    virtual std::string parameters() const override;

    void addListener(DataListener* l);

    template <typename T>
    void addListener(std::function<T(void)> fn, const std::string& name) {
        listeners.push_back(new FunctionListener<T>(name, fn));
    }

    void setFilename(const std::string& filename);
    void setSeparator(const std::string& separator);

    // maybe it's useless?
    void reset();

    virtual bool logToFile() const override;

protected:
    std::string filename, separator, prefix;
    unsigned int counter;
    std::vector<DataListener*> listeners;

    void toFile(std::ostream& o) const;

    void deleteall();

};

// connections
template <typename T>
void connect(NodeOut<T>& out, DataLogger& logger, const std::string& name) {

    DataListener* l = new NodeListener<T>(name, &out);

    logger.addListener(l);

}

template <typename T1, typename T2, typename F>
void connect(NodeOut<T1>& out, DataLogger& logger, const std::string& name, F fun) {

    DataListener* l = new NodeListener<T2>(name, new LinkFunction<T1, T2>(&out, fun));

    logger.addListener(l);

}

}

#endif // DATALOGGER_H
