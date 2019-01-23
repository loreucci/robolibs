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

#ifndef NESTDATAINTERFACES_H
#define NESTDATAINTERFACES_H

#include <vector>

#include <boost/python.hpp>


namespace nest {

// An interface for injecting data in the simulation.
// Data should come from another node and can be sent with
// something like nest.SetStatus
// execute() will be called before the simulation step.
class DataInjector {

public:
    virtual bool connected() const = 0;

    virtual void refreshInputs() = 0;

    virtual void execute() = 0;

};


// An interface for receiving data from the simulation.
// Can read data with nest.GetStatus and store it for other nodes.
// execute() will be called after the simulation step.
class DataReceiver {

public:
    virtual void execute() = 0;

};


class HasGIDList {

public:
    explicit HasGIDList(const boost::python::tuple& gids);
    explicit HasGIDList(const boost::python::list& gids);
    explicit HasGIDList(const std::vector<unsigned int>& gidsv);
    explicit HasGIDList(std::initializer_list<unsigned int> l);

protected:
    boost::python::list gids;

};


class StatusSetter : public DataInjector, public HasGIDList {

public:
    using HasGIDList::HasGIDList;

    virtual void execute() final override;

protected:
    boost::python::dict params;

    virtual void generateParams() = 0;

};


class StatusGetter : public DataReceiver, public HasGIDList {

public:
    using HasGIDList::HasGIDList;

    virtual void execute() final override;

protected:
    boost::python::dict params;

    virtual void consumeParams() = 0;

};

}


#endif // NESTDATAINTERFACES_H
