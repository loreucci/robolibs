#ifndef PLOTTINGCLIENT_H
#define PLOTTINGCLIENT_H

#include <vector>
#include <utility>

#include <QObject>
#include <QtNetwork/QLocalSocket>
#include <QDataStream>

#include <utilities/vector.h>

#include "node.h"
#include "nodelink.h"
#include "controller.h"
#include "simplenodes.h"


namespace sec {

class PlottingClient : public QObject, public Node {

    Q_OBJECT

public:
    explicit PlottingClient(double freq = 0.0, QObject *parent = 0);
    virtual ~PlottingClient();

    void addConnection(NodeOut<double>* out, const QString& name);
    void addVectorConnection(NodeOut<Utils::Vector>* out, const QString& name, unsigned int idx);

    void clear();

    void disconnect();

//    virtual void setFrequency(double freq);

    virtual void refreshInputs();

    virtual bool connected() const;

    virtual void execute();

    virtual std::string parameters() const;

protected:
    std::vector<std::pair<unsigned int, NodeIn<double>>> inputs;
    std::vector<std::tuple<unsigned int, NodeIn<Utils::Vector>, unsigned int>> inputsvec;
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

template <typename T>
void connect(DictionaryNode<T>& source, const std::string& out, PlottingClient& sink, const std::string& name) {

    sink.addConnection(&(source.output(out)), QString(name.c_str()));

}

template <typename C1>
void connect(C1& source, NodeOut<Utils::Vector> C1::* out, const std::vector<unsigned int>& indexes, PlottingClient& sink, const std::vector<std::string>& names) {

    if (indexes.size() != names.size()) {
        throw std::runtime_error("Plottingserver::connect: indexes and names lenght do not match.");
    }

    for (unsigned int i = 0; i < names.size(); i++) {
        sink.addVectorConnection(&(source.*out), QString(names[i].c_str()), indexes[i]);
    }

}

}

#endif // PLOTTINGCLIENT_H
