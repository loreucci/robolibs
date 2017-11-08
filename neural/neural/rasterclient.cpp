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
//        newValue(in.first, in.second);
//        auto ser = serialize(in.getData());
//        std::cout << ser.toStdString() << std::endl;
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
//    QDataStream serial(&ret, QIODevice::WriteOnly);

    for (const Spike& s : data) {
//        serial << id << s.neuron_id << s.time;
        ret += QString::number(id) + " " + QString::number(s.neuron_id) + " " + QString::number(s.time) + " ";
//        ret += " ";
//        ret += s.time;
//        ret += " ";
    }

    return ret;

}

}

void sec::connect(neural::SpikeNodeOut& out, neural::RasterClient& sink, const std::__cxx11::string& name) {

    sink.addConnection(&out, name);

}
