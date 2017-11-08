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

}
