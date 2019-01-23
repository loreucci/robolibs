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

#ifndef SOCKETDATAEXCHANGE_H
#define SOCKETDATAEXCHANGE_H

#include <string>
#include <vector>
#include <unordered_map>

#include <QLocalServer>
#include <QLocalSocket>
#include <QEventLoop>

#include "simplenodes.h"
#include "nodelink.h"
#include "qtclientserverbase.h"


namespace sec {


class SocketDataIn : public QObject, public DictionaryNode<double> {

    Q_OBJECT

public:
    SocketDataIn(const std::string& socketname, const std::vector<std::string>& expected, const std::vector<double> values, double freq = 0.0);

    void addExpectedInput(const std::string& name, double val);

    virtual void waitForStart();

    virtual void execute() override;

    virtual std::string parameters() const override;

protected:
    std::string socketname;
    std::vector<std::string> expected;
    std::vector<double> values;
    bool startsignaled;
    QLocalServer* server;
    QLocalSocket* socket;
    QEventLoop* loop;

    void response(const QString& response);

private slots:
    void acceptConnection();
    void readMessage();

};


class SocketDataOut : public QtClientBase, public DictionaryNode<double> {

    Q_OBJECT

public:
    SocketDataOut(const std::string& socketname, double freq = 0.0);

    void waitForServer();

    void sendStart();

    std::vector<std::pair<std::string, double>> getExpected();

    bool connected() const override;

    virtual void execute() override;

    virtual std::string parameters() const override;

protected:
    std::string socketname;
    std::vector<std::string> expected;
    std::vector<double> values;

};

}


#endif // SOCKETDATAEXCHANGE_H
