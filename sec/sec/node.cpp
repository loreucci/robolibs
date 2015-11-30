#include "node.h"

#include "controller.h"

namespace sec {

unsigned int Node::nextid = 0;

Node::Node(double freq)
    :ID(nextid++) {

    if (freq < 0) {
        std::invalid_argument("Controller: frequence must be non-negative.");
    }
    this->freq = freq;

    main_controller.addNode(this);

}

Node::~Node() {}

void Node::setFrequency(double freq) {

    if (freq < 0) {
        std::invalid_argument("Controller: frequence must be non-negative.");
    }

    double old_freq = this->freq;
    this->freq = freq;
    main_controller.moveNode(this, old_freq);

}

double Node::getFrequency() const {
    return freq;
}

}
