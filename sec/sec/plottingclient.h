#ifndef PLOTTINGCLIENT_H
#define PLOTTINGCLIENT_H

#include <vector>
#include <utility>

#include <QObject>
#include <QtNetwork/QLocalSocket>
#include <QDataStream>

#include "node.h"
#include "nodelink.h"
#include "controller.h"


namespace sec {

class PlottingClient : public QObject, public Node {

    Q_OBJECT

public:
    explicit PlottingClient(double freq = 0.0, QObject *parent = 0);
    virtual ~PlottingClient();

    void addConnection(NodeOut<double>* out, const QString& name);

    void clear();

    void disconnect();

//    virtual void setFrequency(double freq);

    virtual void refreshInputs();

    virtual bool connected() const;

    virtual void execute();

    virtual std::string parameters() const;

protected:
    std::vector<std::pair<unsigned int, NodeIn<double>>> inputs;
    QLocalSocket* socket;

    unsigned int addGraph(const QString& name);

//    void newValue(unsigned int id, double val);

    void changeFreq(double freq);

    void advance();

    void write(const QString& str);

    QString read();

};


template <class C1>
void connect(C1& source, NodeOut<double> C1::* out, PlottingClient& sink, const std::string& name) {

    sink.addConnection(&(source.*out), QString(name.c_str()));

}

}

#endif // PLOTTINGCLIENT_H
