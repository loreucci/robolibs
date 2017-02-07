#include "basedevices.h"


void SpiNNaker::SpikeReceiver::execute() {
    datamutex.lock();
    output.addData(data);
    datamutex.unlock();
}
