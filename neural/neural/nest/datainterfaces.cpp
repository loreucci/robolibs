#include "datainterfaces.h"

#include <stdexcept>

namespace py = boost::python;

namespace nest {

HasGIDList::HasGIDList(const boost::python::tuple& gids) {

    this->gids = py::list(gids);

    if (py::len(this->gids) == 0)
        throw std::invalid_argument("ParameterSetter/Getter: cannot be executed on empty list.");

}

HasGIDList::HasGIDList(const boost::python::list& gids)
    :gids(gids) {

    if (py::len(gids) == 0)
        throw std::invalid_argument("ParameterSetter/Getter: cannot be executed on empty list.");

}

HasGIDList::HasGIDList(const std::vector<unsigned int>& gidsv) {
    for (auto v : gidsv) {
        gids.append(v);
    }

    if (py::len(gids) == 0)
        throw std::invalid_argument("ParameterSetter/Getter: cannot be executed on empty list.");
}

HasGIDList::HasGIDList(std::initializer_list<unsigned int> l)
    :HasGIDList(std::vector<unsigned int>(l)) {}

void StatusSetter::execute() {

    generateParams();

    py::object cmd = "nest.SetStatus(%s, %s)" % py::make_tuple(gids, params);
    std::string cmd_str = py::extract<std::string>(cmd);  // this may be needed because of a bug of boost 1.65
    py::exec(cmd_str.c_str(), main_namespace);
}


void StatusGetter::execute() {

    py::object cmd = "nest.GetStatus(%s)[0]" % gids;
    std::string cmd_str = py::extract<std::string>(cmd);  // this may be needed because of a bug of boost 1.65
    py::object result = py::eval(cmd_str.c_str(), main_namespace);

    params = py::extract<py::dict>(result);

    consumeParams();

}

void DataInjector::setNamespace(const boost::python::api::object& main_namespace) {
    this->main_namespace = main_namespace;
}

void DataReceiver::setNamespace(const boost::python::api::object& main_namespace) {
    this->main_namespace = main_namespace;
}

}
