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

#ifndef DATALISTENER_H
#define DATALISTENER_H

#include <functional>
#include <vector>

#include <utilities/utilities.h>

#include "nodelink.h"


namespace sec {

class DataListener {

public:

    DataListener(const std::string& name = "")
        :name(name) {}

    virtual ~DataListener(){}

    virtual void read() = 0;

    virtual std::string getLine(unsigned int line, const std::string& separator) = 0;

    std::string getName() const {
        return name;
    }

    virtual void reset() = 0;

protected:
    std::string name;

};


template <typename T>
class ValueListener : public DataListener {

public:
    ValueListener(const std::string& name, T& val)
        :DataListener(name), val(val) { }

    virtual void read() override {
        values.push_back(val);
    }

    virtual std::string getLine(unsigned int line, const std::string& separator) {
        return Utils::make_string(values[line], separator);
    }

    virtual void reset() override {
        values.clear();
    }

protected:
    T& val;
    std::vector<T> values;

};


template <typename T>
class NodeListener : public DataListener {

public:
    NodeListener(const std::string& name, NodeOut<T>* nodelink)
        :DataListener(name), nodelink(nodelink) { }

    virtual void read() override {
        values.push_back(nodelink->getData().first);
    }

    virtual std::string getLine(unsigned int line, const std::string& separator) {
        return Utils::make_string(values[line], separator);
    }

    virtual void reset() override {
        values.clear();
    }

protected:
    NodeOut<T>* nodelink;
    std::vector<T> values;

};


template <typename T>
class FunctionListener : public DataListener {

public:

    using funtype = std::function<T(void)>;

    FunctionListener(const std::string& name, funtype fun)
        :DataListener(name), fun(fun){ }

    virtual void read() override {
        values.push_back(fun());
    }

    virtual std::string getLine(unsigned int line, const std::string& separator) {
        return Utils::make_string(values[line], separator);
    }

    virtual void reset() override {
        values.clear();
    }

protected:
    funtype fun;
    std::vector<T> values;

};


}

#endif // DATALISTENER_H

