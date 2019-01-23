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

#include "spikes.h"

namespace neural {


void SpikeData::append(const SpikeData& data) {

    this->insert(this->end(), data.begin(), data.end());

}


SpikeNodeIn::SpikeNodeIn(const SpikeNodeOut* nodeout)
    :mtx(new std::mutex()), nodeout(nodeout), lastid(0), isnew(false) {
    buffereddata.clear();
    bufferedid = 0;
}

void SpikeNodeIn::connect(const SpikeNodeOut* nodeout) {
    this->nodeout = nodeout;
}

bool SpikeNodeIn::isConnected() const {
    return nodeout != nullptr;
}

void SpikeNodeIn::pushData(const SpikeData& newdata, unsigned int newdataid) {
    std::lock_guard<std::mutex> lock(*mtx);
//    mtx->lock();
    buffereddata.insert(buffereddata.end(), newdata.begin(), newdata.end());
    bufferedid = newdataid;
//    mtx->unlock();
}

void SpikeNodeIn::refreshData() {
    if (!isConnected())
        return;

    std::lock_guard<std::mutex> lock(*mtx);
//    mtx->lock();
    if (bufferedid == lastid) {
        isnew = false;
    } else {
        isnew = true;
        data = buffereddata;
        buffereddata.clear();
        lastid = bufferedid;
    }
//    mtx->unlock();
}

SpikeNodeIn::operator SpikeData() const {
    return getData();
}

SpikeData SpikeNodeIn::getData() const {
    std::lock_guard<std::mutex> lock(*mtx);
//    mtx->lock();
    SpikeData ret = data;
//    mtx->unlock();
    return ret;
}

bool SpikeNodeIn::isNew() const {
    return isnew;
}



SpikeNodeOut::SpikeNodeOut()
    :dataid(0) {}

SpikeNodeOut& SpikeNodeOut::operator=(const SpikeData& data) {
    addData(data);
    return *this;
}

void SpikeNodeOut::addData(const SpikeData& data) {
    this->data = data;
    dataid++;
    for (auto link : links)
        link->pushData(data, dataid);
}

std::pair<SpikeData, unsigned int> SpikeNodeOut::getData() const {
    return std::make_pair(data, dataid);
}

void SpikeNodeOut::addConnection(SpikeNodeIn* link) {
    links.push_back(link);
}

}

void sec::connect(neural::SpikeNodeOut* out, neural::SpikeNodeIn* in) {
    in->connect(out);
    out->addConnection(in);
}

void sec::connect(neural::SpikeNodeOut& out, neural::SpikeNodeIn& in) {
    in.connect(&out);
    out.addConnection(&in);
}
