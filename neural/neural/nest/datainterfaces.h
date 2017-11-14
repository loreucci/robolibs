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
