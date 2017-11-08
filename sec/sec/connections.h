#ifndef CONNECTIONS_H
#define CONNECTIONS_H

#include <functional>

#include <utilities/utilities.h>

#include "nodelink.h"
#include "simplesources.h"


namespace sec {


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

// implicitly creates a constant source, only works if default frequency is set
template <typename T>
ConstantSource<T>* connect(const T& val, NodeIn<T>& in) {
    ConstantSource<T>* s = new ConstantSource<T>(val);
    in.connect(&(s->output));
    return s;
}


}

#endif // CONNECTIONS_H
