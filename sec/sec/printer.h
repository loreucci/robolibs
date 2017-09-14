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


////////////////////////////////////////
/// New connection style
template <typename T>
void connect(NodeOut<T>& out, Printer& printer, const std::string& sep = "") {

    auto fun = [&out, sep] () {
        return Utils::make_string(out.getData().first, sep);
    };

    printer.addFun(fun);

}

////////////////////////////////////////
/// All of these will be deprecated soon
template <class C1, typename T>
void connect(C1& source, NodeOut<T> C1::* out, Printer& printer, const std::string& sep = "") {

    auto fun = [&source, out, sep] () {
        return Utils::make_string((source.*out).getData().first, sep);
    };

    printer.addFun(fun);

}

}

#endif // PRINTER_H
