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

    virtual void refreshInputs() override;

    virtual bool connected() const override;

    virtual void execute() override;

    virtual std::string parameters() const override;

    void suppressOutput();

    boost::python::tuple getPopulationGIDs(const std::string& pop);

protected:
    const std::string& nestscript;
    std::vector<DataInjector*> datain;
    std::vector<DataReceiver*> dataout;
    boost::python::dict populations;
    bool popinit;

    void retrievePopulations();

};

}

#endif // NESTEXECNODE_H
