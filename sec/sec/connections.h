#ifndef CONNECTIONS_H
#define CONNECTIONS_H

#include <functional>

#include <utilities/utilities.h>

#include "nodelink.h"
#include "controller.h"


namespace sec {

////////////////////////////////////////
/// New connection style

template <typename T>
void connect(NodeOut<T>& out, NodeIn<T>& in) {
    in.connect(&out);
}

template <typename T, typename F>
void connect(NodeOut<T>& out, NodeIn<T>& in, F fun) {
    in.connect(new LinkFunction<T, T>(&out, fun));
}

template <typename T1, typename T2>
void connect(NodeOut<T1>& out, NodeIn<T2>& in) {
    in.connect(new LinkConverter<T1, T2>(&out));
}

template <typename T1, typename T2, typename F>
void connect(NodeOut<T1>& out, NodeIn<T2>& in, F fun) {
    in.connect(new LinkFunction<T1, T2>(&out, fun));
}


////////////////////////////////////////
/// All of these will be deprecated soon

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
template <class C1, class C2, typename T>
void connect(C1& source, NodeOut<T> C1::* out, C2& sink, NodeIn<T> C2::* in, std::function<T(T)> fun) {

    (sink.*in).connect(new LinkFunction<T, T>(&(source.*out), fun));
    main_controller.registerConnection(&source, &sink);

}


// TODO: this does not work with subclasses of OneToOneNode, investigate...
template <class C1, class C2, typename T1, typename T2>
void connect(C1& source, NodeOut<T1> C1::* out, C2& sink, NodeIn<T2> C2::* in) {

    (sink.*in).connect(new LinkConverter<T1, T2>(&(source.*out)));
    main_controller.registerConnection(&source, &sink);

}

// TODO: this does not work with subclasses of OneToOneNode, investigate...
template <class C1, class C2, typename T1, typename T2>
void connect(C1& source, NodeOut<T1> C1::* out, C2& sink, NodeIn<T2> C2::* in, std::function<T2(T1)> fun) {

    (sink.*in).connect(new LinkFunction<T1, T2>(&(source.*out), fun));
    main_controller.registerConnection(&source, &sink);

}


}

#endif // CONNECTIONS_H

