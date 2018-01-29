#include "qtclientserverbase.h"

#include <iostream>


namespace sec {

QtClientBase::QtClientBase(QObject* parent)
    :QObject(parent) {

    socket = new QLocalSocket(this);

}

QtClientBase::~QtClientBase() {
    disconnectFromServer();
}

bool QtClientBase::connectToServer(QString socketname) {

    socket->connectToServer(socketname);

    if (!socket->isValid()) {
        std::cerr << "[QtClientBase] Unable to connect to server socket." << std::endl;
        return false;
    }

    return true;

}

bool QtClientBase::connectToServerAndWait(QString socketname) {
    socket->connectToServer(socketname);
    return socket->waitForConnected();
}

void QtClientBase::disconnectFromServer() {
    socket->disconnectFromServer();
}

void QtClientBase::write(const QString& str) {

    if (!socket->isValid())
        return;

    QByteArray msg;
    QDataStream out(&msg, QIODevice::WriteOnly);
    out << str;

    socket->write(msg);
    socket->flush();

}

QString QtClientBase::readResponse() {

    if (!socket->isValid())
        return "ERROR_SOCKET_NOT_OPEN";

    socket->waitForReadyRead();

    QDataStream in(socket);

    QString response;
    in >> response;

    return response;

}

bool QtClientBase::waitForConnected() {
    return socket->waitForConnected();
}

}
