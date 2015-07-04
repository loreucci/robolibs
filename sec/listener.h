#ifndef LISTENER_H
#define LISTENER_H

#include <vector>
#include <functional>
#include <string>

#include <utilities/utilities.h>
#include <utilities/message.h>

#include "source.h"
#include "controller.h"

class Listener {

public:

    Listener(const std::string& name = "")
        :name(name) {}

    virtual ~Listener(){}

    virtual void read() = 0;

    virtual std::string getLine(unsigned int line, const std::string& separator) = 0;

    std::string getName() const {
        return name;
    }

protected:
    std::string name;

};




///////////////////
// Common Listeners

template <typename T>
class ValueListener : public Listener {

public:
    ValueListener(const std::string& name, T& val)
        :Listener(name), val(val) { }

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
class FunctionListener : public Listener {

public:

    using funtype = std::function<T(void)>;

    FunctionListener(const std::string& name, funtype fun)
        :Listener(name), fun(fun){ }

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

// helpers
template <typename C, typename T>
FunctionListener<T>* getFunctionListener(const std::string& name, const C& c, T(C::* fn)(void) const) {

    return new FunctionListener<T>(name, [fn, &c](){return (c.*fn)();});

}

template <typename T>
FunctionListener<T>* getFunctionListener(const std::string& name, const Source<T>& s) {
    return new FunctionListener<T>(name, [&](){return s.getCurrent();});
}

template <typename A, typename B>
FunctionListener<B>* getFunctionListener(const std::string& name, const Controller<A, B>& c) {
    return new FunctionListener<B>(name, [&](){return c.getOutput();});
}



template <typename T, unsigned int S, unsigned int ID>
class MessageListener : public Listener {

public:
    MessageListener(const std::string& name, Message<T, S, ID>& msg)
        :Listener(name), msg(msg) { }

    virtual void read() override {
        messages.push_back(msg);
    }

    virtual std::string getLine(unsigned int line, const std::string& separator) {
        return Utils::make_string(messages[line], separator);
    }

protected:
    Message<T, S, ID>& msg;
    std::vector<Message<T, S, ID>> messages;

};

#endif // LISTENER_H
