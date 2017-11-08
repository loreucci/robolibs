#ifndef RASTERCLIENT_H
#define RASTERCLIENT_H

#include <vector>
#include <utility>

#include <sec/node.h>
#include <sec/nodelink.h>
#include <sec/controller.h>
#include <sec/qtclientserverbase.h>

#include "spikes.h"


namespace neural {

class RasterClient : public sec::QtClientBase, public sec::Node {

    Q_OBJECT

public:
    explicit RasterClient(double freq = 0.0, QObject *parent = 0);
    virtual ~RasterClient();

    void addConnection(SpikeNodeOut* out, const std::string& name);

    void clear();

    virtual void refreshInputs();

    virtual bool connected() const;

    virtual void execute();

    virtual std::string parameters() const;

protected:
    std::vector<std::pair<unsigned int, SpikeNodeIn*>> inputs;

    void changeFreq(double freq);

    void advance();

//    void sendSerialized(unsigned int id, const SpikeData& data);

    QString serialize(unsigned int id, const SpikeData& data);

};

}

namespace sec {

void connect(neural::SpikeNodeOut& out, neural::RasterClient& sink, const std::string& name);

}

#endif // RASTERCLIENT_H
