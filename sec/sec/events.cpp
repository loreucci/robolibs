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
