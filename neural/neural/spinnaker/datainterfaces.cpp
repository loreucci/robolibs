/*
 * Copyright (C) 2014-2019 Lorenzo Vannucci
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

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
        throw std::runtime_error("[SpiNNaker::DataInjector] Expected label " + population_name + ", got " + label);

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

DataReceiver::DataReceiver(const std::string& population_name, unsigned int IDoffset)
    :population_name(population_name), IDoffset(IDoffset) {}

std::string DataReceiver::getPopulationName() const {
    return population_name;
}

void DataReceiver::receive_spikes(char* label, int time, int n_spikes, int* spikes) {

    // check that we are in the right thread
    if (std::string(label) != population_name)
        throw std::runtime_error("[SpiNNaker::DataReceiver] Expected label " + population_name + ", got " + label);

    neural::SpikeData newdata;
    for (int spike = 0;  spike < n_spikes; spike++) {
//        printf("Received spike at time %d, from %s - %d \n", time, label, spikes[spike]);
        newdata.push_back({(unsigned int)spikes[spike]+IDoffset, (double)time/1000.0});
        // TODO                                                      ^^^^^^
    }

    datamutex.lock();
    data.clear();
    if (!newdata.empty()) {
        data.insert(data.end(), newdata.begin(), newdata.end());
    }
    datamutex.unlock();

}

}
