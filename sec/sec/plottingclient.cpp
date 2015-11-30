#include "plottingclient.h"


// TODO error management

namespace sec {

PlottingClient::PlottingClient(double freq, QObject *parent) :
    QObject(parent), Node(freq) {

    socket = new QLocalSocket(this);

    socket->connectToServer("plotserver");

    clear();

    changeFreq(freq);

}

PlottingClient::~PlottingClient() {
    disconnect();
}

void PlottingClient::addConnection(NodeOut<double>* out, const QString& name) {

    unsigned int id = addGraph(name);
    inputs.push_back(std::make_pair(id, NodeIn<double>(out)));

}

//void PlottingClient::setFrequency(double freq) {
//    if (freq < 0) {
//        std::invalid_argument("Controller: frequence must be non-negative.");
//    }

//    double old_freq = this->freq;
//    this->freq = freq;
//    main_controller.moveNode(this, old_freq);

//    changeFreq(freq);

//}

void PlottingClient::refreshInputs() {
    for (auto& in : inputs) {
        in.second.refreshData();
    }
}

bool PlottingClient::connected() const {
    for (const auto& in : inputs)
        if (!in.second.isConnected())
            return false;
    return true;
}

void PlottingClient::execute() {
    QString newvalues = "newvalues ";
    for (auto& in : inputs) {
//        newValue(in.first, in.second);
        newvalues += QString::number(in.first) + " " + QString::number(in.second.getData()) + " ";
    }
    newvalues.chop(1);
    write(newvalues);
    read();
    advance();
}

std::string PlottingClient::parameters() const {
    return "Plotting client.";
}

unsigned int PlottingClient::addGraph(const QString& name) {

    write(QString("addgraph ") + name);
    return read().toInt();

}

//void PlottingClient::newValue(unsigned int id, double val) {

//    write("newvalue " + QString::number(id) + " " + QString::number(val));
//    read();

//}

void PlottingClient::clear() {
    write("clearall");
    read();
}

void PlottingClient::changeFreq(double freq) {
    write("setfreq " + QString::number(freq));
    read();
}

void PlottingClient::advance() {
    write("advance");
    read();
}

void PlottingClient::disconnect() {
    socket->disconnectFromServer();
}

void PlottingClient::write(const QString& str) {

    QByteArray msg;
    QDataStream out(&msg, QIODevice::WriteOnly);
    out << str;

    socket->write(msg);
    socket->flush();

}

QString PlottingClient::read() {

    socket->waitForReadyRead();

    QDataStream in(socket);

    QString response;
    in >> response;

    return response;

}

}
