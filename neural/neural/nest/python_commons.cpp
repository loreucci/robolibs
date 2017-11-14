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
