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
    ExecutionNode(const std::string& nestscript, const std::vector<DataInjector*>& datain = {}, const std::vector<DataReceiver*>& dataout = {}, double freq = 100.0);

    void addDataInOut(const std::vector<DataInjector*>& datain, const std::vector<DataReceiver*>& dataout);
    void addDataIn(const std::vector<DataInjector*>& datain);
    void addDataOut(const std::vector<DataReceiver*>& dataout);

//    virtual ~ExecutionNode();

    virtual void refreshInputs() override;

    virtual bool connected() const override;

    virtual void execute() override;

    virtual std::string parameters() const override;

    static void initPythonRuntime();

    void suppressOutput();

    boost::python::tuple getPopulationGIDs(const std::string& pop);

protected:
    const std::string& nestscript;
    std::vector<DataInjector*> datain;
    std::vector<DataReceiver*> dataout;
    boost::python::dict populations;

    static boost::python::object main_namespace;

};

}

#endif // NESTEXECNODE_H
