#include "execnode.h"
#include <iostream>

namespace py = boost::python;


namespace nest {

ExecutionNode::ExecutionNode(const std::string& nestscript, std::vector<DataInjector*> datain, std::vector<DataReceiver*> dataout, double freq)
    :Node(freq), nestscript(nestscript), datain(datain), dataout(dataout) {

    try {
        auto ret = py::exec_file(nestscript.c_str(), main_namespace);
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
    py::exec((py::str)cmd, main_namespace);

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

    py::exec("nest.sli_run('M_WARNING setverbosity')", main_namespace);

}

py::object ExecutionNode::main_namespace;

}
