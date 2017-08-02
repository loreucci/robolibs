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

void PlottingClient::addConnection(LinkSource<double>* out, const QString& name, std::function<double(double)> fun) {

    unsigned int id = addGraph(name);
    inputs.push_back(std::make_tuple(id, NodeIn<double>(out), fun));

}

void PlottingClient::addVectorConnection(LinkSource<Utils::Vector>* out, const QString& name, unsigned int idx, std::function<double(double)> fun) {

    unsigned int id = addGraph(name);
    inputsvec.push_back(std::make_tuple(id, NodeIn<Utils::Vector>(out), idx, fun));

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
//        newValue(in.first, in.second);
        newvalues += QString::number(std::get<0>(in)) + " " + QString::number(std::get<2>(in)(std::get<1>(in).getData())) + " ";
    }
    for (auto& in : inputsvec) {
        newvalues += QString::number(std::get<0>(in)) + " " + QString::number(std::get<3>(in)(std::get<1>(in).getData()[std::get<2>(in)])) + " ";
    }
    newvalues.chop(1);
    write(newvalues);
    readResponse();
    advance();
}

std::string PlottingClient::parameters() const {
    return "Plotting client.";
}

unsigned int PlottingClient::addGraph(const QString& name) {

    write(QString("addgraph ") + name);
    return readResponse().toInt();

}

//void PlottingClient::newValue(unsigned int id, double val) {

//    write("newvalue " + QString::number(id) + " " + QString::number(val));
//    read();

//}

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

void PlottingClient::disconnect() {
    socket->disconnectFromServer();
}

//void PlottingClient::setFrequency(double freq) {
//    Node::setFrequency(freq);
//    changeFreq(freq);
//}

void PlottingClient::write(const QString& str) {

    QByteArray msg;
    QDataStream out(&msg, QIODevice::WriteOnly);
    out << str;

    socket->write(msg);
    socket->flush();

}

QString PlottingClient::readResponse() {

    socket->waitForReadyRead();

    QDataStream in(socket);

    QString response;
    in >> response;

    return response;

}

std::function<double(double)> identity_fun = [](double x){return x;};

void connect(NodeOut<double>& out, PlottingClient& sink, const std::string& name, std::function<double (double)> fun) {

    sink.addConnection(&out, QString(name.c_str()), fun);

}

}
