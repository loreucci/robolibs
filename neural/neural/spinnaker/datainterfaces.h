#ifndef SPINNAKERDATAINTERFACES_H
#define SPINNAKERDATAINTERFACES_H

#include <string>
#include <vector>
#include <mutex>
#include <condition_variable>

#include <SpynnakerLiveSpikesConnection.h>

#include "../spikes.h"

namespace SpiNNaker {

// An interface for injecting data in the simulation.
// It relies on SpikesStartCallbackInterface, so only
// vectors of int can be sent.
class DataInjector : public SpikesStartCallbackInterface {

public:
    explicit DataInjector(const std::string& population_name);

    std::string getPopulationName() const;

    virtual bool connected() const = 0;

    virtual void refreshInputs() = 0;

    // should prepare the data vector using NodeIns and wake up the other thread by
    // cv.notify_one();
    virtual void execute() = 0;

    void stop();

    virtual void spikes_start(char *label, SpynnakerLiveSpikesConnection *connection) override;

protected:
    std::vector<int> data;
    std::mutex mtx;
    std::condition_variable cv;
    bool quit = false;

    std::string population_name;

};


class DataReceiver : public SpikeReceiveCallbackInterface {

public:
    DataReceiver(const std::string& population_name, unsigned int IDoffset = 0);

    std::string getPopulationName() const;

    // should use data, with mutex access (and maybe empty it)
    virtual void execute() = 0;

    virtual void receive_spikes(char *label, int time, int n_spikes, int* spikes) override;

protected:
    std::mutex datamutex;
    neural::SpikeData data;
    unsigned int IDoffset;

    std::string population_name;

};

}


#endif // SPINNAKERDATAINTERFACES_H
