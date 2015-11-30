#include "printer.h"

#include <iostream>


namespace sec {

Printer::Printer(const std::string& sep)
    :Node(0.0), sep(sep) {

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
        std::cout << "No input connection to printer.";

    for (unsigned i = 0; i < funs.size()-1; i++) {
        std::cout << funs[i]() << sep;
    }
    std::cout << funs.back()() << std::endl;

}

std::string Printer::parameters() const {
    return "Global printer.";
}

}
