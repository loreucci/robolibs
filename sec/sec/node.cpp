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

#include "node.h"

#include "controller.h"
#include "commons.h"

namespace sec {

unsigned int Node::nextid = 0;

Node::Node(double freq)
    :ID(nextid++), delay(0.0) {

    if (freq < 0) {
        std::invalid_argument("Controller: frequence must be non-negative.");
    }

    if (freq == 0.0)
        this->freq = getDefaultFrequency();
    else
        this->freq = freq;

    main_controller.addNode(this);

}

Node::~Node() {

    main_controller.removeNode(this);

}

void Node::setFrequency(double freq) {

    if (freq < 0) {
        std::invalid_argument("Controller: frequence must be non-negative.");
    }

    double old_freq = this->freq;
    if (freq == 0.0)
        this->freq = getDefaultFrequency();
    else
        this->freq = freq;
    main_controller.moveNode(this, old_freq);

}

double Node::getFrequency() const {
    return freq;
}

void Node::runOnSingleThread() {
    main_controller.moveNodeToSingleThread(this);
}

std::string Node::parametersShort() const {

    std::string par = parameters();
    if (par.length() <= 20) {
        return par;
    }
    par.resize(17);
    auto p = par.find('\n');
    if (p != std::string::npos) {
        par.resize(p);
    }
    par += "...";
    return par;

}

void Node::setDelay(double delay) {
    this->delay = delay;
}

double Node::getDelay() const {
    return delay;
}

void Node::reset() {}

}
