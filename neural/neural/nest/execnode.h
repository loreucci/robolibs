#ifndef NESTEXECNODE_H
#define NESTEXECNODE_H

#include <string>
#include <vector>

#include <sec/node.h>

#include <boost/python.hpp>

#include "datainterfaces.h"


namespace nest {

class ExecutionNode : public sec::Node {

public:
    ExecutionNode(const std::string& nestscript, std::vector<DataInjector*> datain, std::vector<DataReceiver*> dataout, double freq = 100.0);

//    virtual ~ExecutionNode();

    virtual void refreshInputs() override;

    virtual bool connected() const override;

    virtual void execute() override;

    virtual std::string parameters() const override;

    static void initPythonRuntime();

    void suppressOutput();

protected:
    const std::string& nestscript;
    std::vector<DataInjector*> datain;
    std::vector<DataReceiver*> dataout;

    static boost::python::object main_namespace;

};

}

#endif // NESTEXECNODE_H
