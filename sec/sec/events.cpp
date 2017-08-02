#include "events.h"

#include "synchronization.h"

namespace sec {


Event::Event(Event::Funtype fun, double time)
    :fun(fun), time(time) {

}


void EventManager::addEvent(Event::Funtype fun, double time) {
    events.push_back(Event(fun, time));
}

void EventManager::refreshInputs() {}

bool EventManager::connected() const {
    return true;
}

void EventManager::execute() {

    double time = synchronizer.getTime();

    for (auto e = events.begin(); e != events.end();) {

        if (e->time <= time) {
            e->fun();
            e = events.erase(e);
        } else {
            ++e;
        }

    }

}

std::string EventManager::parameters() const {
    return "EventManager";
}

}
