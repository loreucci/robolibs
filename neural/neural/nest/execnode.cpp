#include "execnode.h"
#include <iostream>

namespace py = boost::python;


namespace nest {

ExecutionNode::ExecutionNode(const std::string& nestscript, const std::vector<DataInjector*>& datain, const std::vector<DataReceiver*>& dataout, double freq)
    :Node(freq), nestscript(nestscript), datain(datain), dataout(dataout) {

    try {
        py::exec_file(nestscript.c_str(), main_namespace);
    } catch (py::error_already_set const &) {
        PyErr_Print();
    }

    // try to get parameters
    try {
        py::object result = py::eval("populations", main_namespace);
        populations = py::extract<py::dict>(result);
    } catch (py::error_already_set const &) {
        PyErr_Print();
    }

    for (auto din : datain) {
        din->setNamespace(main_namespace);
    }

    for (auto dout : dataout) {
        dout->setNamespace(main_namespace);
    }

}

void ExecutionNode::addDataInOut(const std::vector<DataInjector*>& datain, const std::vector<DataReceiver*>& dataout) {

    for (auto din : datain) {
        din->setNamespace(main_namespace);
    }

    for (auto dout : dataout) {
        dout->setNamespace(main_namespace);
    }

    this->datain.insert(this->datain.end(), datain.begin(), datain.end());
    this->dataout.insert(this->dataout.end(), dataout.begin(), dataout.end());

}

void ExecutionNode::addDataIn(const std::vector<DataInjector*>& datain) {
    addDataInOut(datain, {});
}

void ExecutionNode::addDataOut(const std::vector<DataReceiver*>& dataout) {
    addDataInOut({}, dataout);
}

//ExecutionNode::~ExecutionNode() {}

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
    py::object cmd = "nest.Simulate(%.1f)" % py::make_tuple(1000.0/getFrequency());
    std::string cmd_str = py::extract<std::string>(cmd);  // this may be needed because of a bug of boost 1.65
    py::exec(cmd_str.c_str(), main_namespace);

    for (auto dout : dataout) {
        dout->execute();
    }

}

std::string ExecutionNode::parameters() const {
    return "NEST node executing " + nestscript;
}

void ExecutionNode::initPythonRuntime() {
    Py_Initialize();
    char *argv[1] = {""};
    PySys_SetArgv(1, argv);

    auto main_module = py::import("__main__");
    main_namespace = main_module.attr("__dict__");

}

void ExecutionNode::suppressOutput() {

    try {
        py::exec("nest.sli_run('M_WARNING setverbosity')", main_namespace);
    } catch (py::error_already_set const &) {
        PyErr_Print();
    }
}

boost::python::tuple ExecutionNode::getPopulationGIDs(const std::string& pop) {

    return py::extract<py::tuple>(populations[pop]);

}

py::object ExecutionNode::main_namespace;

}
