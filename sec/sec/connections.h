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
