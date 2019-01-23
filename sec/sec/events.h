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
