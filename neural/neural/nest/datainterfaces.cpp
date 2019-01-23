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
