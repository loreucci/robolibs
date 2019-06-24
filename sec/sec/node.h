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

//!  \file node.h
/*!
  This file contains the Node class.
*/

#ifndef NODE_H
#define NODE_H

#include <string>
#include <stdexcept>

namespace sec {

//!  Node class.
/*!
  This class represent a node of the controller.
*/
class Node {

public:
    Node(double freq = 0.0);

    // Node is not MoveConstructible, CopyConstructible, MoveAssignable or CopyAssignable
    Node(Node&&) = delete;
    Node(const Node&) = delete;
    Node& operator=(Node&&) = delete;
    Node& operator=(const Node&) = delete;

    virtual ~Node();

    virtual void setFrequency(double freq);

    virtual double getFrequency() const final;

    virtual void runOnSingleThread() final;

    virtual void refreshInputs() = 0;

    virtual bool connected() const = 0;

    virtual void execute() = 0;

    virtual std::string parameters() const = 0;
    virtual std::string parametersShort() const final;

    virtual void setDelay(double delay) final;

    virtual double getDelay() const final;

    virtual void reset();

    const unsigned int ID;

protected:
    double freq, delay;

    static unsigned int nextid;

};


}

#endif // NODE_H

