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

#include "printer.h"

#include <iostream>


namespace sec {

Printer::Printer(const std::string& sep, double freq)
    :Node(freq), sep(sep) {

}

void Printer::addFun(Printer::FunType fun) {
    funs.push_back(fun);
}

void Printer::refreshInputs() {}

bool Printer::connected() const {
    return true;
}

void Printer::execute() {

    if (funs.empty())
        return;

    for (unsigned i = 0; i < funs.size()-1; i++) {
        std::cout << funs[i]() << sep;
    }
    std::cout << funs.back()() << std::endl;

}

std::string Printer::parameters() const {
    return "Global printer.";
}

}

