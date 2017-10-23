#include "socketdataexchange.h"

#include <stdexcept>
#include <QDataStream>
#include <iostream>


namespace sec {


SocketDataIn::SocketDataIn(const std::string& socketname, const std::vector<std::string>& expected, const std::vector<double> values, double freq)
    :QObject(), DictionaryNode<double>({}, {}, freq), socketname(socketname), expected(expected), values(values) {

    // set dictionary
    if (expected.size() != values.size()) {
        throw  std::invalid_argument("SocketDataIn: expected names and values sizes must agree.");
    }

    for (unsigned int i = 0; i < expected.size(); i++) {
        auto it = outdict.insert({expected[i], NodeOut<double>()}).first;
        (*it).second = values[i];
    }

    // open socket
    startsignaled = false;
    socket = nullptr;
    server = new QLocalServer(this);

    if (!server->listen(socketname.c_str())) {
        throw  std::runtime_error("SocketDataIn: unable to run server: " + server->errorString().toStdString());
    }

    connect(server, SIGNAL(newConnection()), this, SLOT(acceptConnection()));


    loop = new QEventLoop(this);

}

void SocketDataIn::addExpectedInput(const std::string& name, double val) {

    auto it = outdict.insert({name, NodeOut<double>()}).first;
    (*it).second = val;
    expected.push_back(name);
    values.push_back(val);

}

void SocketDataIn::waitForStart() {

    while (!startsignaled) {
        loop->processEvents();
        sec::synchronizer.sleep(10);
    }

}

void SocketDataIn::execute() {

    loop->processEvents();

}

std::string SocketDataIn::parameters() const {
    return "Socket data input on " + socketname;
}

void SocketDataIn::response(const QString& response) {

    QByteArray msg;
    QDataStream out(&msg, QIODevice::WriteOnly);
    out << response;

    socket->write(msg);
    socket->flush();

}

void SocketDataIn::acceptConnection() {

    socket = server->nextPendingConnection();
    connect(socket, SIGNAL(disconnected()), socket, SLOT(deleteLater()));
    connect(socket, SIGNAL(readyRead()), this, SLOT(readMessage()));

}

void SocketDataIn::readMessage() {

    QDataStream in(socket);

    QString nextMsg;
    in >> nextMsg;

    QStringList cmd = nextMsg.split(" ");

    if (cmd[0] == "GET_EXPECTED") {

        QString res("BEGIN_EXPECTED ");

        for (unsigned int i = 0; i < expected.size(); i++) {

            res += QString::fromStdString(expected[i]);
            res += " ";
            res += QString::number(values[i]);
            res += " ";

        }

        res += "END_EXPECTED";

        response(res);

    } else if (cmd[0] == "BEGIN_VALUES") {

        for (unsigned int i = 1; i < cmd.size()-1; i+=2){
            output(cmd[i].toStdString()) = cmd[i+1].toDouble();
        }

        response("OK");

    } else if (cmd[0] == "START") {

        startsignaled = true;
        response("OK");

    } else {

        std::cerr << "SocketDataIn: unrecognized command\n";

        response("ERROR");

    }

}


SocketDataOut::SocketDataOut(const std::string& socketname, double freq)
    :QtClientBase(), DictionaryNode<double>({}, {}, freq), socketname(socketname) {

}

void SocketDataOut::waitForServer() {

    for (unsigned int i = 0; i < 3000; i++) {
        if (connectToServerAndWait(QString::fromStdString(socketname)))
            break;
        synchronizer.sleep(10);
    }

    // get expected values
    write("GET_EXPECTED");
    QString expvalues = readResponse();

    QStringList cmd = expvalues.split(" ");
    if (cmd[0] == "BEGIN_EXPECTED") {

        for (unsigned int i = 1; i < cmd.size()-1; i+=2) {
            std::string n = cmd[i].toStdString();
            expected.push_back(n);
            values.push_back(cmd[i+1].toDouble());
            indict.insert({n, NodeIn<double>()});
        }

    } else {
        throw std::runtime_error("SocketDataIn: received wrong string.");
    }

}

void SocketDataOut::sendStart() {
    write("START");
    readResponse();
}

std::vector<std::pair<std::string, double>> SocketDataOut::getExpected() {

    std::vector<std::pair<std::string, double>> ret;

    for (unsigned int i = 0; i < expected.size(); i++) {
        ret.push_back(std::make_pair(expected[i], values[i]));
    }

    return ret;

}

bool SocketDataOut::connected() const {
    return true;
}

void SocketDataOut::execute() {

    QString tosend("BEGIN_VALUES ");
    bool sendsomething = false;
    for (auto& in : indict) {
        if (in.second.isConnected()) {
            sendsomething = true;
            tosend += QString::fromStdString(in.first);
            tosend += " ";
            tosend += QString::number(in.second.getData());
            tosend += " ";
        }
    }
    tosend += "END_VALUES";

    if (sendsomething) {
        write(tosend);
        readResponse();
    }

}

std::string SocketDataOut::parameters() const {
    return "Socket data out on " + socketname;
}


}
