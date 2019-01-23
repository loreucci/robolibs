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

#ifndef SIMPLENODES_H
#define SIMPLENODES_H

#include <deque>
#include <vector>
#include <unordered_map>
#include <utility>
#include <functional>

#include <utilities/utilities.h>

#include "node.h"
#include "nodelink.h"
#include "controller.h"
#include "printer.h"
#include "datalogger.h"


namespace sec {

class DummyNode : public Node {

public:

    using Node::Node;

    virtual void refreshInputs() override {}

    virtual void execute() override {}

    virtual std::string parameters() const override {
        return "Dummy node.";
    }

    virtual bool connected() const override {
        return true;
    }

};


// DEBUG ONLY ?
template <typename In, typename Out>
class OneToOneNode : public Node {

public:
    using Node::Node;

    virtual void refreshInputs() final override {
        input.refreshData();
    }

    virtual bool connected() const final override {
        return input.isConnected();
    }

    // DEBUG ONLY
    virtual void execute() override {
        output.addData(Out());
    }

    virtual std::string parameters() const override {
        return "Generic OneToOne node.";
    }

    NodeIn<In> input;
    NodeOut<Out> output;


};


template <typename T>
class Delay : public Node {

public:
    Delay(double delay, double freq = 0.0)
        :Node(freq), tdelay(delay) {
        mem = std::deque<double>();
    }

    virtual void refreshInputs() final override {
        input.refreshData();
    }

    virtual bool connected() const final override {
        return input.isConnected();
    }

    virtual void execute() override {

        if (mem.empty()) {
            unsigned int memsize = tdelay*getFrequency();
            mem.resize(memsize, T());
        }

        mem.push_back(input);
        output = mem.front();
        mem.pop_front();
    }

    virtual std::string parameters() const override {
        return "Delay of " + std::to_string(tdelay) + "s.";
    }

    NodeIn<T> input;
    NodeOut<T> output;

protected:
    double tdelay;
    std::deque<double> mem;

};


template <typename T>
class DictionaryNode : public Node {

public:
    DictionaryNode(const std::vector<std::string>& inputs, const std::vector<std::string>& outputs, double freq = 0.0)
        :Node(freq) {

        for (const auto& in : inputs) {
            indict.insert(std::make_pair(in, NodeIn<T>()));
        }

        for (const auto& out : outputs) {
            outdict.insert(std::make_pair(out, NodeOut<T>()));
        }

    }

    virtual void refreshInputs() {
        for (auto& in : indict) {
            in.second.refreshData();
        }
    }

    virtual bool connected() const {
        for (const auto& in : indict) {
            if (!in.second.isConnected())
                return false;
        }
        return true;
    }

    NodeIn<T>& input(const std::string& idx) {
        return indict.at(idx);
    }

    NodeOut<T>& output(const std::string& idx) {
        return outdict.at(idx);
    }

protected:
    std::unordered_map<std::string, NodeIn<T>> indict;
    std::unordered_map<std::string, NodeOut<T>> outdict;

};

}

#endif // SIMPLENODES_H
