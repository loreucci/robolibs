#ifndef CONNECTIONS_H
#define CONNECTIONS_H

#include <utilities/utilities.h>

#include "simplenodes.h"
#include "nodelink.h"
#include "printer.h"
#include "controller.h"
#include "logger.h"


namespace sec {

// this will not register the connection,
// but it may be useful if the links are not
// in nodes
template <typename T>
void connect(NodeOut<T>* out, NodeIn<T>* in) {
    in->connect(out);
}

template <class C1, class C2>
void connect(C1& source, C2& sink) {

    sink.input.connect(&(source.output));
    main_controller.registerConnection(&source, &sink);

}

// TODO: this may be dangerous
template <class C1, class C2, typename T>
void connect(C1& source, NodeOut<T>* out, C2& sink, NodeIn<T>* in) {

    in->connect(out);
    main_controller.registerConnection(&source, &sink);

}

// TODO: this does not work with subclasses of OneToOneNode, investigate...
template <class C1, class C2, typename T>
void connect(C1& source, NodeOut<T> C1::* out, C2& sink, NodeIn<T> C2::* in) {

    (sink.*in).connect(&(source.*out));
    main_controller.registerConnection(&source, &sink);

}

// TODO: this does not work with subclasses of OneToOneNode, investigate...
template <class C1, class C2, typename T1, typename T2>
void connect(C1& source, NodeOut<T1> C1::* out, C2& sink, NodeIn<T2> C2::* in) {

    (sink.*in).connect(new LinkConverter<T1, T2>(&(source.*out)));
    main_controller.registerConnection(&source, &sink);

}

// TODO: move to printer.h
template <class C1, typename T>
void connect(C1& source, NodeOut<T> C1::* out, Printer& printer, const std::string& sep = "") {

    auto fun = [&source, out, sep] () {
        return Utils::make_string((source.*out).getData().first, sep);
    };

    printer.addFun(fun);

}

// TODO: move to logger.h
template <class C1, typename T>
void connect(C1& source, NodeOut<T> C1::* out, Logger& logger, const std::string& name) {

    Listener* l = new NodeListener<T>(name, &(source.*out));

    logger.addListener(l);

}


// TODO: move this in a separate file (with DictionaryNode)
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

// dict to printer
template <typename T>
void connect(DictionaryNode<T>& source, const std::string& out, Printer& printer, const std::string& sep = "") {

    auto fun = [&source, out, sep] () {
        return Utils::make_string((source.input(out)).getData().first, sep);
    };

    printer.addFun(fun);

}

// dict to logger
template <typename T>
void connect(DictionaryNode<T>& source, const std::string& out, Logger& logger, const std::string& name) {

    Listener* l = new NodeListener<T>(name, &(source.output(out)));

    logger.addListener(l);

}


}

#endif // CONNECTIONS_H

