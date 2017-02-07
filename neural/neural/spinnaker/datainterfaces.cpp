#include "datainterfaces.h"

#include <stdexcept>

namespace SpiNNaker {

DataInjector::DataInjector    (const std::string& population_name)
    :population_name(population_name) {}

std::string DataInjector::getPopulationName() const {
    return population_name;
}

void DataInjector::stop() {
    quit = true;
    cv.notify_one();
}

void DataInjector::spikes_start(char* label, SpynnakerLiveSpikesConnection* connection) {

    // check that we are in the right thread
    if (std::string(label) != population_name)
        throw std::runtime_error("SpinnakerDataInjector: expected label " + population_name + ", got " + label);

    while (true) {
        // wait for the signal to send data
        std::unique_lock<std::mutex> lk(mtx);
        cv.wait(lk);
        lk.unlock();

        // check if the thread needs to exit
        if (quit)
            break;

        // if there is data to be sent, send it
        if (!data.empty())
            connection->send_spikes(label, data);

    }

}

DataReceiver::DataReceiver(const std::string& population_name)
    :population_name(population_name) {}

std::string DataReceiver::getPopulationName() const {
    return population_name;
}

void DataReceiver::receive_spikes(char* label, int time, int n_spikes, int* spikes) {

    // check that we are in the right thread
    if (std::string(label) != population_name)
        throw std::runtime_error("SpinnakerDataReceiver: expected label " + population_name + ", got " + label);

    neural::SpikeData newdata;
    for (int spike = 0;  spike < n_spikes; spike++) {
//        printf("Received spike at time %d, from %s - %d \n", time, label, spikes[spike]);
        newdata.push_back({(unsigned int)spikes[spike], (double)time/1000.0});
        // TODO                                                      ^^^^^^
    }

    if (!newdata.empty()) {
        datamutex.lock();
        data.insert(data.end(), newdata.begin(), newdata.end());
        datamutex.unlock();
    }

}

}
