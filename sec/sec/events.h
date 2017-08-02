#ifndef EVENTS_H
#define EVENTS_H

#include <functional>
#include <list>

#include "node.h"


namespace sec {


// a timed event
struct Event {

    using Funtype = std::function<void(void)>;

    explicit Event(Funtype fun, double time);

    Funtype fun;

    double time;

};


class EventManager : public Node {

public:
    using Node::Node;

    void addEvent(Event::Funtype fun, double time);

    template <typename C>
    void addEvent(C& obj, void (C::*fun)(void), double time) {

        auto f = [&obj, fun] { (obj.*fun)(); };
        events.push_back(Event(f, time));

    }

    virtual void refreshInputs() override;

    virtual bool connected() const override;

    virtual void execute() override;

    virtual std::string parameters() const override;

protected:
    std::list<Event> events;

};

}

#endif // EVENTS_H
