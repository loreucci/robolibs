#ifndef LOGGER_H
#define LOGGER_H

#include <string>

#include "node.h"
#include "listener.h"


namespace sec {

class Logger : public Node {

public:
    Logger(const std::string& basename = "test", const std::string& separator = " ", const std::string& extension = ".txt");

    virtual ~Logger();

    virtual void refreshInputs() override;

    virtual bool connected() const override;

    virtual void execute() override;

    virtual std::string parameters() const override;

    void addListener(Listener* l);

    template <typename T>
    void addListener(std::function<T(void)> fn, const std::string& name) {
        listeners.push_back(new FunctionListener<T>(name, fn));
    }

    void setBasename(const std::string& basename);
    void setExtension(const std::string& extension);
    void setSeparator(const std::string& separator);

    void reset();

    void toggleLogging(bool toggle = false);

    void saveToFile() const;

    void logNodes(std::vector<Node*> nodes) const;

protected:
    std::string basename, separator, extension, timestamp;
    bool enabled;
    unsigned int counter;
    std::vector<Listener*> listeners;

    void toFile(std::ostream& o) const;

    void deleteall();

};

}

#endif // LOGGER_H
