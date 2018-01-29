#include "datainterfaces.h"

#include <stdexcept>

#include "python_commons.h"

namespace py = boost::python;

namespace nest {

HasGIDList::HasGIDList(const boost::python::tuple& gids) {

    this->gids = py::list(gids);

    if (py::len(this->gids) == 0)
        throw std::invalid_argument("[ParameterSetter/Getter] Cannot be created from empty list.");

}

HasGIDList::HasGIDList(const boost::python::list& gids)
    :gids(gids) {

    if (py::len(gids) == 0)
        throw std::invalid_argument("[ParameterSetter/Getter] Cannot be created from empty list.");

}

HasGIDList::HasGIDList(const std::vector<unsigned int>& gidsv) {
    for (auto v : gidsv) {
        gids.append(v);
    }

    if (py::len(gids) == 0)
        throw std::invalid_argument("[ParameterSetter/Getter]: cannot be created from empty list.");
}

HasGIDList::HasGIDList(std::initializer_list<unsigned int> l)
    :HasGIDList(std::vector<unsigned int>(l)) {}

void StatusSetter::execute() {

    generateParams();

    py_exec("nest.SetStatus(%s, %s)" % py::make_tuple(gids, params));

}


void StatusGetter::execute() {

    params = py_eval<py::dict>("nest.GetStatus(%s)[0]" % gids);

    consumeParams();

}

}
