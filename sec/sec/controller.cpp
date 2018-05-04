#include "controller.h"

#include <algorithm>
#include <csignal>
#include <iostream>

#include "resultscollector.h"
#include "flags.h"
#include "commons.h"


namespace sec {

unsigned int escalation = 0;
void handler(int) {
    synchronizer.quitAll();
    if (sec::isVerbose()) {
        std::cerr << "[Controller] Sending shutdown" << std::endl;
    }
    escalation++;
    if (escalation >= 3) {
        if (sec::isVerbose()) {
            std::cerr << "[Controller] Escalating to SIGKILL" << std::endl;
        }
        std::raise(SIGKILL);
    }
}


Controller::Controller() {

    maxfreq = 0.0;

}

Controller::~Controller() {

    std::vector<Node*> todel;
    for (auto& bucket : nodes) {
        for (auto& n : bucket.second) {
            todel.push_back(n);
        }
        bucket.second.clear();
    }
    nodes.clear();

    for (const auto& n : singleThreadNodes) {
        todel.push_back(n.second);
    }
    singleThreadNodes.clear();

    for (unsigned int i = 0; i < todel.size(); i++) {
        delete todel[i];
    }

}

void Controller::addNode(sec::Node* node) {

    if (node->getFrequency() > maxfreq)
        maxfreq = node->getFrequency();

    nodes[node->getFrequency()].push_front(node);

    if (sec::isVerbose()) {
        std::cerr << "[Controller] Added node " << node->ID << " @ " << node->getFrequency() << "Hz" << std::endl;
    }

}

void Controller::moveNode(Node* node, double old_freq) {

    if (nodes.find(old_freq) != nodes.end()) {
        nodes[old_freq].remove_if([node](sec::Node* n){return n->ID == node->ID;});
    } else if (std::find(singleThreadNodes.begin(), singleThreadNodes.end(), std::make_pair(old_freq, node)) != singleThreadNodes.end()) {
        auto it = std::find(singleThreadNodes.begin(), singleThreadNodes.end(), std::make_pair(old_freq, node));
        (*it).first = node->getFrequency();
    } else {
        throw std::runtime_error("[Controller] Node not found in list, cannot be moved.");
    }

    if (sec::isVerbose()) {
        std::cerr << "[Controller] Moving node " << node->ID << " from " << old_freq << "Hz" << std::endl;
    }

    addNode(node);
}

void Controller::removeNode(Node* node) {

    for (auto& bucket : nodes) {
        bucket.second.remove_if([node](sec::Node* n){return n->ID == node->ID;});
    }
    auto pos = std::remove_if(singleThreadNodes.begin(), singleThreadNodes.end(), [node](std::pair<double, Node*> p){return p.second->ID == node->ID;});
    singleThreadNodes.erase(pos, singleThreadNodes.end());

    if (sec::isVerbose()) {
        std::cerr << "[Controller] Removing node " << node->ID << std::endl;
    }

}

void Controller::sortNodes() {

    // remove empty lists
    for(auto it = nodes.begin(); it != nodes.end();) {
      if (it->second.empty()) {
        it = nodes.erase(it);
      }
      else
        ++it;
    }

    // actual sort
    for (auto& bucket : nodes) {
        bucket.second.sort([](sec::Node* n1, sec::Node* n2){return n1->ID < n2->ID;});
    }
}

std::pair<bool, std::vector<Node*>> Controller::checkConnections() const {
    std::vector<Node*> disc;
    for (const auto& bucket : nodes) {
        for (const auto& n : bucket.second) {
            if (!n->connected())
                disc.push_back(n);
        }
    }
    for (const auto& n : singleThreadNodes) {
        if (!n.second->connected())
            disc.push_back(n.second);
    }
    return std::make_pair(disc.empty(), disc);
}

void Controller::moveNodeToSingleThread(Node* node) {

    nodes[node->getFrequency()].remove(node);

    singleThreadNodes.push_back(std::make_pair(node->getFrequency(), node));

}


void Controller::executeThread(ExecThread* et) {

    bool endcond = false;
    double maxtime = 1.0 / et->nodes.front()->getFrequency();
    double threadtime = 0.0;

    while (!et->sem.shouldquit() && !endcond) {

        et->sem.wait();

        auto start = std::chrono::system_clock::now();

        // exec nodes
        for (auto n : et->nodes) {
            if (n->getDelay() > threadtime) {
                continue;
            }
            n->refreshInputs();
            n->execute();
        }

        // check for termination conditions
        for (auto& endfun : et->endconditions)
            if (endfun())
                endcond = true;

        if (synchronizer.isSynchronous()) {
            et->sem.completion_notify(false);
        } else {
            // check if the thread execution time exceeded max
            std::chrono::duration<double> diff = std::chrono::system_clock::now()-start;
            if (diff.count() > maxtime) {
                std::cerr << "Warning: this thread is too slow to run @ " << et->nodes.front()->getFrequency() << "Hz.";
                std::cerr << " (actual: " << 1.0/diff.count() << "Hz)\n";
            }
        }

        threadtime += 1.0 / et->nodes.front()->getFrequency();

    }

    if (synchronizer.isSynchronous()) {
        et->sem.completion_notify(true);
    }

}

void Controller::promoteNodesToDefault() {

    // check if there is something to do
    if (nodes[0.0].empty())
        return;

    // if no default frequency was specified die
    double freq = getDefaultFrequency();
    if (freq == 0.0) {
        std::string disc;
        for (const auto& n : nodes[0.0])
            disc += n->parametersShort() + "\n";
        throw std::runtime_error("[Controller] No frequency was specified for the following nodes:\n\n" + disc);
    }

    // otherwise se the nodes to default frequency
    // it is not an infinite loop because setFrequency actually removes the node from the list
    while (!nodes[0.0].empty()) {
        nodes[0.0].front()->setFrequency(freq);
    }

}

void Controller::run(double time, std::vector<std::function<bool(void)>> endconditions) {

    // register signal handlers
    std::signal(SIGINT, handler);
    std::signal(SIGTERM, handler);

    // check everything
    auto connected = checkConnections();
    if (!connected.first) {
        std::string disc;
        for (const auto& n : connected.second)
            disc += n->parametersShort() + "\n";
        throw std::runtime_error("[Controller] Some nodes are not connected:\n\n" + disc);
    }

    // promote nodes with 0.0 to default frequency
    promoteNodesToDefault();

    // sort nodes to the order of creation
    sortNodes();

    // create thread specs
    std::deque<ExecThread> threads;

    for (auto it : singleThreadNodes) {
        ExecThread th{nullptr,
                      {it.second},
                      synchronizer.registerSignal(it.first),
                      std::forward_list<std::function<bool(void)>>()};
        threads.push_back(th);
    }

    for (auto it : nodes) {
        ExecThread th{nullptr,
                      it.second,
                      synchronizer.registerSignal(it.first),
                      std::forward_list<std::function<bool(void)>>()};
        threads.push_back(th);
    }

    if (threads.empty())
        throw std::runtime_error("[Controller] Nothing to run.");

    // sort threads
    std::sort(threads.begin(), threads.end(), [](const ExecThread& t1, const ExecThread& t2){
        return t1.nodes.front()->getFrequency() < t2.nodes.front()->getFrequency();
    });

    // verbose output
    if (sec::isVerbose()) {
        std::cerr << "\n[Controller] About to start the following threads:" << std::endl;
        unsigned tid = 0;
        for (auto& t : threads) {
            std::cerr << "- thread " << tid << ":" <<std::endl;
            for (auto n : t.nodes) {
                std::cerr << "\tID = " << n->ID << ", " << n->parametersShort() << std::endl;
            }
            tid++;
            std::cerr << std::endl;
        }
    }

    // check for same frequency in synchronous mode
    if (synchronizer.isSynchronous()) {
        for (unsigned int i = 0; i < threads.size()-1; i++) {
            if (threads[i].nodes.front()->getFrequency() != threads[i+1].nodes.front()->getFrequency())
                throw std::runtime_error("[Controller] Synchronous mode is not compatible with multi-frequencies executions.");
        }
    }

    // create threads
    for (unsigned int i = 0; i < threads.size()-1; i++) {
        threads[i].t = new std::thread(&Controller::executeThread, this, &threads[i]);
    }

    // add termination conditions to main thread
    for (auto& tc : endconditions)
        threads.back().endconditions.push_front(tc);

    // create termination condition for time
    unsigned int count = 0;
    unsigned int maxcount = time * threads.back().nodes.front()->getFrequency();
    auto timefun = [&count, maxcount] () {
        count++;
        return count >= maxcount;
    };
    if (time > 0.0)
        threads.back().endconditions.push_front(timefun);

    // run main
    synchronizer.start();
    executeThread(&threads.back());
    synchronizer.stop();
    synchronizer.quitAll();
    synchronizer.unregisterAll();

    // join all
    for (auto& th : threads) {
        if (th.t != nullptr)
            th.t->join();
    }

    // save results
    results_collector.saveAll();

    if (sec::isVerbose()) {
        std::cerr << "[Controller] Threads ended" << std::endl;
    }

    // unregister signal handlers
    std::signal(SIGINT, SIG_DFL);
    std::signal(SIGTERM, SIG_DFL);

}

std::vector<Node*> Controller::getAllNodes() {
    std::vector<Node*> ret;
    for (const auto& it : nodes) {
        for (const auto n : it.second) {
            ret.push_back(n);
        }
    }

    for (const auto& it : singleThreadNodes) {
        ret.push_back(it.second);
    }

    return ret;
}

Controller main_controller = Controller();

}
