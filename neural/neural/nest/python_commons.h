#ifndef PYTHON_COMMON_H
#define PYTHON_COMMON_H

#include <string>

#include <boost/python.hpp>


namespace nest {

// wrappers around boost::python eval and exec functions
// this is done to avoid having to do string conversions and try/catches every time
// and to avoid having to pass the scope every time
// the string convertions may be needed because of a bug of boost 1.65

extern boost::python::object main_namespace;

template <typename T>
T py_eval(boost::python::object cmd) {

    T ret;

    try {
        std::string cmd_str = boost::python::extract<std::string>(cmd);
        boost::python::object result = boost::python::eval(cmd_str.c_str(), main_namespace);
        ret = boost::python::extract<T>(result);
    } catch (boost::python::error_already_set&) {
        PyErr_Print();
    }

    return ret;

}

template <typename T>
T py_eval(const char* cmd) {

    T ret;

    try {
        boost::python::object result = boost::python::eval(cmd, main_namespace);
        ret = boost::python::extract<T>(result);
    } catch (boost::python::error_already_set&) {
        PyErr_Print();
    }

    return ret;

}

void py_exec(boost::python::object cmd);
void py_exec(const char* cmd);

void py_exec_file(const char* filename);

}

#endif // PYTHON_COMMON_H
