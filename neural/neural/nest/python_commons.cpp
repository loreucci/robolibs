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

#include "python_commons.h"

namespace py = boost::python;

namespace nest {


py::object initPythonRuntime() {
    Py_Initialize();
    const char *argv[1] = {""};
    PySys_SetArgv(1, const_cast<char**>(argv));  // no way to deal with this without emitting a warning or doing this cast T.T

    auto main_module = py::import("__main__");
    py::object main_namespace = main_module.attr("__dict__");

    return main_namespace;

}
py::object main_namespace = initPythonRuntime();


void py_exec(py::object cmd) {

    try {
        std::string cmd_str = py::extract<std::string>(cmd);
        py::exec(cmd_str.c_str(), main_namespace);
    } catch (py::error_already_set const &) {
        PyErr_Print();
    }

}

void py_exec(const char* cmd) {

    try {
        py::exec(cmd, main_namespace);
    } catch (py::error_already_set const &) {
        PyErr_Print();
    }

}

void py_exec_file(const char* filename) {

    try {
        py::exec_file(filename, main_namespace);
    } catch (py::error_already_set const &) {
        PyErr_Print();
    }

}

}
