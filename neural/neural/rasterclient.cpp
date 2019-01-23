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

#include "rasterclient.h"


#include <iostream>
// TODO error management

namespace neural {

RasterClient::RasterClient(double freq, QObject *parent) :
    sec::QtClientBase(parent), sec::Node(freq) {

    connectToServer("spikeserver");

    clear();

    changeFreq(freq);

}

RasterClient::~RasterClient() {
    disconnect();
    for (auto in : inputs)
        delete in.second;
}

void RasterClient::addConnection(SpikeNodeOut* out, const std::string& name) {

    write(QByteArray("addgraph ").append(QString(name.c_str())));
    unsigned int id = readResponse().toInt();
    inputs.push_back(std::make_pair(id, new SpikeNodeIn(out)));
    out->addConnection(inputs.back().second);

}


void RasterClient::refreshInputs() {
    for (auto& in : inputs) {
        in.second->refreshData();
    }
}

bool RasterClient::connected() const {
    for (const auto& in : inputs)
        if (!in.second->isConnected())
            return false;
    return true;
}

void RasterClient::execute() {
    QString newvalues = "newvalues ";

    bool tosend = false;

    for (auto& in : inputs) {
        if (in.second->isNew()) {
            newvalues += serialize(in.first, in.second->getData());
            tosend = true;
        }
    }

    newvalues.chop(1);
    if (tosend) {
        write(newvalues);
        readResponse();
    }
    advance();
}

std::string RasterClient::parameters() const {
    return "Raster client.";
}

void RasterClient::clear() {
    write("clearall");
    readResponse();
}

void RasterClient::changeFreq(double freq) {
    write("setfreq " + QString::number(freq));
    readResponse();
}

void RasterClient::advance() {
    write("advance");
    readResponse();
}

QString RasterClient::serialize(unsigned int id, const SpikeData& data) {

    QString ret;

    for (const Spike& s : data) {
        ret += QString::number(id) + " " + QString::number(s.neuron_id) + " " + QString::number(s.time) + " ";
    }

    return ret;

}

}

void sec::connect(neural::SpikeNodeOut& out, neural::RasterClient& sink, const std::string& name) {

    sink.addConnection(&out, name);

}
