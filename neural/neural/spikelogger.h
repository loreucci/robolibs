#ifndef SPIKELOGGER_H
#define SPIKELOGGER_H

#include <sec/node.h>

#include "spikes.h"


namespace neural {


class SpikeLogger : public sec::Node {

public:
    SpikeLogger(const std::string& basename = "test", const std::string& extension = ".txt");

    virtual ~SpikeLogger();

    virtual void refreshInputs() override;

    virtual bool connected() const override;

    virtual void execute() override;

    virtual std::string parameters() const override;

    void addSpikeSource(SpikeNodeOut* source);

    void saveToFile() const;

    void toggleLogging(bool toggle = false);


protected:
    std::string basename, extension, timestamp;
    std::vector<SpikeNodeIn*> listeners;
    bool enabled;

    SpikeData data;

};


}

namespace sec {

void connect(neural::SpikeNodeOut* out, neural::SpikeLogger& sink);

template <class C1>
void connect(C1& source, neural::SpikeNodeOut C1::* out, neural::SpikeLogger& sink) {

    sink.addSpikeSource(&(source.*out));

}

}

#endif // SPIKELOGGER_H
