#ifndef SPINNAKERBASEDEVICES_H
#define SPINNAKERBASEDEVICES_H

#include "datainterfaces.h"

#include "../spikes.h"


namespace SpiNNaker {


class SpikeReceiver : public DataReceiver {

public:
    using DataReceiver::DataReceiver;

    virtual void execute() override;

    neural::SpikeNodeOut output;

};


}

#endif // SPINNAKERBASEDEVICES_H
