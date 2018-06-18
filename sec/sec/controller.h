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
    ~Controller();

    void addNode(Node* node);
    void moveNode(Node* node, double old_freq);
    void removeNode(Node* node);

    void moveNodeToSingleThread(Node* node);

    void run(double time = 0.0, std::vector<std::function<bool(void)>> endconditions = {});

    void resetAllNodes();

    // TODO: make an iterator
    std::vector<Node*> getAllNodes();

protected:
    std::unordered_map<double, std::forward_list<Node*>> nodes;
    std::vector<std::pair<double, Node*>> singleThreadNodes;
    double maxfreq;

    std::pair<bool, std::vector<Node*>> checkConnections() const;

    void sortNodes();

    void executeThread(ExecThread* et);

    void promoteNodesToDefault();

};

extern Controller main_controller;

}

#endif // CONTROLLER_H
