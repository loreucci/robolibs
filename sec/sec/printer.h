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

#ifndef PRINTER_H
#define PRINTER_H

#include <deque>
#include <functional>
#include <string>

#include <utilities/utilities.h>

#include "node.h"
#include "nodelink.h"

namespace sec {

class Printer : public Node {

public:
    using FunType = std::function<std::string(void)>;

    Printer(const std::string& sep = " ", double freq = 0.0);

    void addFun(FunType fun);

    virtual void refreshInputs() override;

    virtual bool connected() const override;

    virtual void execute() override;

    virtual std::string parameters() const override;


protected:
    std::string sep;
    std::deque<FunType> funs;

};


//connections
template <typename T>
void connect(NodeOut<T>& out, Printer& printer, const std::string& sep = "") {

    auto fun = [&out, sep] () {
        return Utils::make_string(out.getData().first, sep);
    };

    printer.addFun(fun);

}

}

#endif // PRINTER_H
