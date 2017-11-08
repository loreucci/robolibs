#ifndef PLOTTINGCLIENT_H
#define PLOTTINGCLIENT_H

#include <vector>
#include <utility>
#include <functional>

#include <utilities/vector.h>

#include "node.h"
#include "nodelink.h"
#include "controller.h"
#include "simplenodes.h"
#include "qtclientserverbase.h"


namespace sec {

class PlottingClient : public QtClientBase, public Node {

    Q_OBJECT

public:
    explicit PlottingClient(double freq = 0.0, QObject *parent = 0);
    virtual ~PlottingClient();

    void addConnection(LinkSource<double>* out, const QString& name, std::function<double(double)> fun);
    void addVectorConnection(LinkSource<Utils::Vector>* out, const QString& name, unsigned int idx, std::function<double(double)> fun);

    void clear();

    virtual void refreshInputs();

    virtual bool connected() const;

    virtual void execute();

    virtual std::string parameters() const;

protected:
    std::vector<std::tuple<unsigned int, NodeIn<double>, std::function<double(double)>>> inputs;
    std::vector<std::tuple<unsigned int, NodeIn<Utils::Vector>, unsigned int, std::function<double(double)>>> inputsvec;

    unsigned int addGraph(const QString& name);

    void changeFreq(double freq);

    void advance();

};


// this function has to created just once in order to avoid a linking error with the templates
extern std::function<double(double)> identity_fun;

// connections
void connect(NodeOut<double>& out, PlottingClient& sink, const std::string& name, std::function<double(double)> fun = identity_fun);

template <typename T, typename F>
void connect(NodeOut<T>& out, PlottingClient& sink, const std::string& name, F fun) {

    sink.addConnection(new LinkFunction<T, double>(&out, fun), QString(name.c_str()), identity_fun);

}

template <typename T>
void connect(DictionaryNode<T>& source, const std::string& out, PlottingClient& sink, const std::string& name, std::function<double(double)> fun = identity_fun) {

    sink.addConnection(&(source.output(out)), QString(name.c_str()), fun);

}

void connect(NodeOut<Utils::Vector>& out, const std::vector<unsigned int>& indexes, PlottingClient& sink, const std::vector<std::string>& names, std::vector<std::function<double(double)>> funs = {});

}

#endif // PLOTTINGCLIENT_H
