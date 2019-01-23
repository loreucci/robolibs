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
