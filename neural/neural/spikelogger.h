#ifndef SPIKELOGGER_H
#define SPIKELOGGER_H

#include <sec/node.h>
#include <sec/resultscollector.h>

#include "spikes.h"


namespace neural {


class SpikeLogger : public sec::Node, public sec::Logger {

public:
    SpikeLogger(const std::string& filename = "spikes.txt", double freq = 0.0);

    virtual ~SpikeLogger();

    virtual void refreshInputs() override;

    virtual bool connected() const override;

    virtual void execute() override;

    virtual std::string parameters() const override;

    void addSpikeSource(SpikeNodeOut* source);

    bool logToFile() const override;

protected:
    std::string filename, prefix;
    std::vector<SpikeNodeIn*> listeners;

    SpikeData data;

};


}

namespace sec {

void connect(neural::SpikeNodeOut& out, neural::SpikeLogger& sink);

}

#endif // SPIKELOGGER_H
