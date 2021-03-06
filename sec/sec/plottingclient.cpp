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

#include "plottingclient.h"


// TODO error management

namespace sec {

PlottingClient::PlottingClient(double freq, QObject *parent)
    :QtClientBase(parent), Node(freq) {

    connectToServer("plotserver");

    clear();

    changeFreq(this->freq);

}

PlottingClient::~PlottingClient() {
    disconnect();
}

void PlottingClient::addConnection(LinkSource<double>* out, const QString& name, std::function<double(double)> fun) {

    unsigned int id = addGraph(name);
    inputs.push_back(std::make_tuple(id, NodeIn<double>(out), fun));

}

void PlottingClient::addVectorConnection(LinkSource<Utils::Vector>* out, const QString& name, unsigned int idx, std::function<double(double)> fun) {

    unsigned int id = addGraph(name);
    inputsvec.push_back(std::make_tuple(id, NodeIn<Utils::Vector>(out), idx, fun));

}

void PlottingClient::refreshInputs() {
    for (auto& in : inputs) {
        std::get<1>(in).refreshData();
    }
    for (auto& in : inputsvec) {
        std::get<1>(in).refreshData();
    }
}

bool PlottingClient::connected() const {
    for (const auto& in : inputs)
        if (!std::get<1>(in).isConnected())
            return false;
    for (auto& in : inputsvec) {
        if (!std::get<1>(in).isConnected())
            return false;
    }
    return true;
}

void PlottingClient::execute() {

    QString newvalues = "newvalues ";
    for (auto& in : inputs) {
        newvalues += QString::number(std::get<0>(in)) + " ";
        // this is f(dataIn)
        newvalues += QString::number(std::get<2>(in)(std::get<1>(in).getData())) + " ";
    }
    for (auto& in : inputsvec) {
        newvalues += QString::number(std::get<0>(in)) + " ";
        // this is f(dataIn[idx])
        newvalues += QString::number(std::get<3>(in)(std::get<1>(in).getData()[std::get<2>(in)])) + " ";
    }
    newvalues.chop(1);
    write(newvalues);
    readResponse();
    advance();
}

std::string PlottingClient::parameters() const {
    return "Plotting client.";
}

void PlottingClient::setFrequency(double freq) {
    changeFreq(freq);
    Node::setFrequency(freq);
}

unsigned int PlottingClient::addGraph(const QString& name) {

    write(QString("addgraph ") + name);
    return readResponse().toInt();

}

void PlottingClient::clear() {
    write("clearall");
    readResponse();
}

void PlottingClient::changeFreq(double freq) {
    write("setfreq " + QString::number(freq));
    readResponse();
}

void PlottingClient::advance() {
    write("advance");
    readResponse();
}


std::function<double(double)> identity_fun = [](double x){return x;};

void connect(NodeOut<double>& out, PlottingClient& sink, const std::string& name, std::function<double (double)> fun) {

    sink.addConnection(&out, QString(name.c_str()), fun);

}

void connect(NodeOut<Utils::Vector>& out, const std::vector<unsigned int>& indexes, PlottingClient& sink, const std::vector<std::string>& names, std::vector<std::function<double(double)> > funs) {

    if (indexes.size() != names.size()) {
        throw std::runtime_error("[PlottingClient::connect] Indexes and names lenght do not match.");
    }

    if (funs.size() == 0) {
        funs.resize(indexes.size(), identity_fun);
    }

    if (indexes.size() != funs.size()) {
        throw std::runtime_error("[Plottingserver::connect] Indexes and transformation functions lenght do not match.");
    }

    for (unsigned int i = 0; i < names.size(); i++) {
        sink.addVectorConnection(&out, QString(names[i].c_str()), indexes[i], funs[i]);
    }

}

}
