#ifndef CONNECTIONS_H
#define CONNECTIONS_H

#include <utilities/utilities.h>

#include "simplenodes.h"
#include "nodelink.h"
#include "printer.h"
#include "controller.h"
#include "logger.h"


namespace sec {


// TODO: this does not work with subclasses of OneToOneNode, investigate...
template <class C1, class C2, typename T>
void connect(C1& source, NodeOut<T> C1::* out, C2& sink, NodeIn<T> C2::* in) {

    (sink.*in).connect(&(source.*out));
    main_controller.registerConnection(&source, &sink);

}

// to printer
template <class C1, typename T>
void connect(C1& source, NodeOut<T> C1::* out, Printer& printer, const std::string& sep = "") {

    auto fun = [&source, out, sep] () {
        return Utils::make_string((source.*out).getData().first, sep);
    };

    printer.addFun(fun);

}

// to logger
template <class C1, typename T>
void connect(C1& source, NodeOut<T> C1::* out, Logger& logger, const std::string& name) {

    Listener* l = new NodeListener<T>(name, &(source.*out));

    logger.addListener(l);

}

// dict nodes specializations
template <class C1, typename T>
void connect(C1& source, NodeOut<T> C1::* out, DictionaryNode<T>& sink, const std::string& in) {

    sink.input(in).connect(&(source.*out));
    main_controller.registerConnection(&source, &sink);

}

template <class C2, typename T>
void connect(DictionaryNode<T>& source, const std::string& out, C2& sink, NodeIn<T> C2::* in) {

    (sink.*in).connect(&(source.output(out)));
    main_controller.registerConnection(&source, &sink);

}

template <typename T>
void connect(DictionaryNode<T>& source, const std::string& out, DictionaryNode<T>& sink, const std::string& in) {

    sink.input(in).connect(&(source.output(out)));
    main_controller.registerConnection(&source, &sink);

}

}

#endif // CONNECTIONS_H

