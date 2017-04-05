#ifndef DATALISTENER_H
#define DATALISTENER_H

#include <functional>
#include <vector>

#include <utilities/utilities.h>

#include "nodelink.h"


namespace sec {

class DataListener {

public:

    DataListener(const std::string& name = "")
        :name(name) {}

    virtual ~DataListener(){}

    virtual void read() = 0;

    virtual std::string getLine(unsigned int line, const std::string& separator) = 0;

    std::string getName() const {
        return name;
    }

protected:
    std::string name;

};


template <typename T>
class ValueListener : public DataListener {

public:
    ValueListener(const std::string& name, T& val)
        :DataListener(name), val(val) { }

    virtual void read() override {
        values.push_back(val);
    }

    virtual std::string getLine(unsigned int line, const std::string& separator) {
        return Utils::make_string(values[line], separator);
    }

protected:
    T& val;
    std::vector<T> values;

};


template <typename T>
class NodeListener : public DataListener {

public:
    NodeListener(const std::string& name, NodeOut<T>* nodelink)
        :DataListener(name), nodelink(nodelink) { }

    virtual void read() override {
        values.push_back(nodelink->getData().first);
    }

    virtual std::string getLine(unsigned int line, const std::string& separator) {
        return Utils::make_string(values[line], separator);
    }

protected:
    NodeOut<T>* nodelink;
    std::vector<T> values;

};


template <typename T>
class FunctionListener : public DataListener {

public:

    using funtype = std::function<T(void)>;

    FunctionListener(const std::string& name, funtype fun)
        :DataListener(name), fun(fun){ }

    virtual void read() override {
        values.push_back(fun());
    }

    virtual std::string getLine(unsigned int line, const std::string& separator) {
        return Utils::make_string(values[line], separator);
    }

protected:
    funtype fun;
    std::vector<T> values;

};


}

#endif // DATALISTENER_H

