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

    virtual void setFrequency(double freq) final;

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

