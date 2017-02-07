#ifndef SPINNAKEREXECNODE_H
#define SPINNAKEREXECNODE_H

#include <string>
#include <vector>

#include <sec/node.h>

#include <SpynnakerLiveSpikesConnection.h>

#include "datainterfaces.h"


namespace SpiNNaker {

class ExecutionNode : public sec::Node, SpikesStartCallbackInterface {

public:
    ExecutionNode(const std::string& pynnscript, std::vector<DataInjector*> datain, std::vector<DataReceiver*> dataout, double freq = 100.0);

    virtual ~ExecutionNode();

    virtual void refreshInputs() override;

    virtual bool connected() const override;

    virtual void execute() override;

    virtual std::string parameters() const override;

    // synchronization with main controller
    virtual void spikes_start(char*, SpynnakerLiveSpikesConnection*) override;
    bool isStarted() const;
    bool processFinished() const;

protected:
    const std::string& pynnscript;
    std::vector<DataInjector*> datain;
    std::vector<DataReceiver*> dataout;

    std::vector<char*> send_labels, receive_labels;

    SpynnakerLiveSpikesConnection* connection;

    bool started = false;
    mutable std::mutex started_mutex;
    mutable bool ended = false;

    pid_t child;

};

}

#endif // SPINNAKEREXECNODE_H
