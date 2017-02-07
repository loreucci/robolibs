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
