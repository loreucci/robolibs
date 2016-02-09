#ifndef PRINTER_H
#define PRINTER_H

#include <deque>
#include <functional>
#include <string>

#include "node.h"

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

}

#endif // PRINTER_H
