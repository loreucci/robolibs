#ifndef NESTDATAINTERFACES_H
#define NESTDATAINTERFACES_H

#include <boost/python.hpp>


namespace nest {

// An interface for injecting data in the simulation.
// Data should come from another node and can be sent with
// something like nest.SetStatus
// execute() will be called before the simulation step.
class DataInjector {

public:
    virtual void setNamespace(const boost::python::object& main_namespace) final;

    virtual bool connected() const = 0;

    virtual void refreshInputs() = 0;

    virtual void execute() = 0;

protected:
    boost::python::object main_namespace;

};


// An interface for receiving data from the simulation.
// Can read data with nest.GetStatus and store it for other nodes.
// execute() will be called after the simulation step.
class DataReceiver {

public:
    virtual void setNamespace(const boost::python::object& main_namespace) final;

    virtual void execute() = 0;

protected:
    boost::python::object main_namespace;

};




class StatusSetter : public DataInjector {

public:
    StatusSetter(const boost::python::list& gids);

    virtual void execute() final override;

protected:
    boost::python::list gids;
    boost::python::dict params;

    virtual void generateParams() = 0;

};


class StatusGetter : public DataReceiver {

public:
    StatusGetter(const boost::python::list& gids);

    virtual void execute() final override;

protected:
    boost::python::list gids;
    boost::python::dict params;

    virtual void consumeParams() = 0;

};

}


#endif // NESTDATAINTERFACES_H
