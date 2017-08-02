#ifndef PLOTTINGCLIENT_H
#define PLOTTINGCLIENT_H

#include <vector>
#include <utility>
#include <functional>

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

    void addConnection(LinkSource<double>* out, const QString& name, std::function<double(double)> fun);
    void addVectorConnection(LinkSource<Utils::Vector>* out, const QString& name, unsigned int idx, std::function<double(double)> fun);

    void clear();

    void disconnect();

//    virtual void setFrequency(double freq);

    virtual void refreshInputs();

    virtual bool connected() const;

    virtual void execute();

    virtual std::string parameters() const;

protected:
    std::vector<std::tuple<unsigned int, NodeIn<double>, std::function<double(double)>>> inputs;
    std::vector<std::tuple<unsigned int, NodeIn<Utils::Vector>, unsigned int, std::function<double(double)>>> inputsvec;
    QLocalSocket* socket;

    unsigned int addGraph(const QString& name);

//    void newValue(unsigned int id, double val);

    void changeFreq(double freq);

    void advance();

    void write(const QString& str);

    QString readResponse();

};


// this function has to created just once in order to avoid a linking error with the templates
extern std::function<double(double)> identity_fun;

////////////////////////////////////////
/// New connection style
void connect(NodeOut<double>& out, PlottingClient& sink, const std::string& name, std::function<double(double)> fun = identity_fun);

template <typename T, typename F>
void connect(NodeOut<T>& out, PlottingClient& sink, const std::string& name, F fun) {

    sink.addConnection(new LinkFunction<T, double>(&out, fun), QString(name.c_str()), identity_fun);

}


////////////////////////////////////////
/// All of these will be deprecated soon
template <class C1>
void connect(C1& source, NodeOut<double> C1::* out, PlottingClient& sink, const std::string& name, std::function<double(double)> fun = identity_fun) {

    sink.addConnection(&(source.*out), QString(name.c_str()), fun);

}

template <class C1>
void connect(C1&, NodeOut<double>* out, PlottingClient& sink, const std::string& name, std::function<double(double)> fun = identity_fun) {

    sink.addConnection(out, QString(name.c_str()), fun);

}

template <typename T>
void connect(DictionaryNode<T>& source, const std::string& out, PlottingClient& sink, const std::string& name, std::function<double(double)> fun = identity_fun) {

    sink.addConnection(&(source.output(out)), QString(name.c_str()), fun);

}

template <typename C1>
void connect(C1& source, NodeOut<Utils::Vector> C1::* out, const std::vector<unsigned int>& indexes, PlottingClient& sink, const std::vector<std::string>& names, std::vector<std::function<double(double)>> funs = {}) {

    if (indexes.size() != names.size()) {
        throw std::runtime_error("Plottingserver::connect: indexes and names lenght do not match.");
    }

    if (funs.size() == 0) {
        funs.resize(indexes.size(), identity_fun);
    }

    if (indexes.size() != funs.size()) {
        throw std::runtime_error("Plottingserver::connect: indexes and transformation functions lenght do not match.");
    }

    for (unsigned int i = 0; i < names.size(); i++) {
        sink.addVectorConnection(&(source.*out), QString(names[i].c_str()), indexes[i], funs[i]);
    }

}

}

#endif // PLOTTINGCLIENT_H
