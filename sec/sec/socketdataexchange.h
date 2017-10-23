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
