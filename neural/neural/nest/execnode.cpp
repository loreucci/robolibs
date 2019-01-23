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

#include "execnode.h"
#include <iostream>

#include "python_commons.h"

namespace py = boost::python;


namespace nest {

ExecutionNode::ExecutionNode(const std::string& nestscript, const std::vector<DataInjector*>& datain, const std::vector<DataReceiver*>& dataout, double freq)
    :Node(freq), nestscript(nestscript), datain(datain), dataout(dataout) {

    py_exec_file(nestscript.c_str());

    // populations not initialized
    popinit = false;

}

void ExecutionNode::addDataInOut(const std::vector<DataInjector*>& datain, const std::vector<DataReceiver*>& dataout) {

    this->datain.insert(this->datain.end(), datain.begin(), datain.end());
    this->dataout.insert(this->dataout.end(), dataout.begin(), dataout.end());

}

void ExecutionNode::addDataIn(const std::vector<DataInjector*>& datain) {
    addDataInOut(datain, {});
}

void ExecutionNode::addDataOut(const std::vector<DataReceiver*>& dataout) {
    addDataInOut({}, dataout);
}

void ExecutionNode::refreshInputs() {
    for (auto din : datain) {
        din->refreshInputs();
    }
}

bool ExecutionNode::connected() const {
    bool conn = true;
    for (auto din : datain) {
        conn = conn && din->connected();
    }
    return conn;
}

void ExecutionNode::execute() {

    for (auto din : datain) {
        din->execute();
    }

    // run simulation
    py_exec("nest.Simulate(%.1f)" % py::make_tuple(1000.0/getFrequency()));

    for (auto dout : dataout) {
        dout->execute();
    }

}

std::string ExecutionNode::parameters() const {
    return "NEST node executing " + nestscript;
}

void ExecutionNode::suppressOutput() {

    py_exec("nest.sli_run('M_WARNING setverbosity')");

}

boost::python::tuple ExecutionNode::getPopulationGIDs(const std::string& pop) {

    if (!popinit)
        retrievePopulations();

    return py::extract<py::tuple>(populations[pop]);

}

void ExecutionNode::retrievePopulations() {

    // try to get populations
    try {
        py::object result = py::eval("populations", main_namespace);
        populations = py::extract<py::dict>(result);
    } catch (py::error_already_set const &) {
        PyErr_Print();
    }

}

}
