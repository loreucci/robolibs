#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <unordered_map>
#include <forward_list>
#include <thread>
#include <functional>
#include "synchronization.h"

#include "node.h"

namespace sec {

struct ExecThread {

    std::thread* t;
    std::forward_list<Node*> nodes;
    Semaphore sem;
    std::forward_list<std::function<bool(void)>> endconditions;

};


class Controller {

public:
    Controller();

    void addNode(Node* node);
    void moveNode(Node* node, double old_freq);
    void registerConnection(Node* source, Node* sink);


    bool checkConnections() const;
    void adjustFrequencies();

    void sortNodes();

    void executeThread(ExecThread* et);
    void executeFreq(double freq);

    void run(double time = 0.0);

    // DEBUG ONLY
    void printNodes() const;
    void printAdj() const;

    // TODO: make an iterator
    std::vector<Node*> getAllNodes();

protected:
    std::unordered_map<double, std::forward_list<Node*>> nodes;
    std::unordered_map<Node*, std::forward_list<Node*>> adj;
    double maxfreq;

};

// TODO: move all singletons on a separate file (sec.h)?
extern Controller main_controller;

}

#endif // CONTROLLER_H
