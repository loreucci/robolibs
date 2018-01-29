#include "execnode.h"
#include <sys/wait.h>
#include <unistd.h>
#include <sec/flags.h>

#include <iostream>


namespace SpiNNaker {

char const* start_label = "startamelo";

ExecutionNode::ExecutionNode(const std::string& pynnscript, std::vector<DataInjector*> datain, std::vector<DataReceiver*> dataout, double freq)
    :Node(freq), pynnscript(pynnscript), datain(datain), dataout(dataout) {

    ended = false;
    started = false;

    // get labels
    for (auto din : datain) {
        unsigned int len = din->getPopulationName().length()+1;
        char* str = new char(len);
        din->getPopulationName().copy(str, len);
        str[len-1] = '\0';
        send_labels.push_back(str);
    }
    for (auto dout : dataout) {
        unsigned int len = dout->getPopulationName().length()+1;
        char* str = new char(len);
        dout->getPopulationName().copy(str, len);
        str[len-1] = '\0';
        receive_labels.push_back(str);
    }

    // TODO                                                                                                                                            vvvvv
    connection = new SpynnakerLiveSpikesConnection(receive_labels.size(), receive_labels.data(), send_labels.size(), send_labels.data(), (char*) NULL, 19999);

    // simulation start signaling
    connection->add_start_callback((char*)start_label, this);

    // data injectors
    for (unsigned int i = 0; i < datain.size(); i++) {
        connection->add_start_callback(send_labels[i], datain[i]);
    }

    // data receivers
    for (unsigned int i = 0; i < dataout.size(); i++) {
        connection->add_receive_callback(receive_labels[i], dataout[i]);
    }


    //    child = 0;
    child = fork();
    if (child == -1) {
        throw std::runtime_error("[SpiNNaker::ExecutionNode] Unable to fork.");
    } else if (child == 0){
        execl("/usr/bin/python2", "python", pynnscript.c_str(), nullptr);
    }

    if (sec::isVerbose())
        std::cerr << "[SpiNNaker::ExecutionNode] Starting simulation" << std::endl;

}

ExecutionNode::~ExecutionNode() {
    for (auto din : datain) {
        din->stop();
    }
    delete connection;
}

void ExecutionNode::refreshInputs() {
    for (auto din : datain) {
        din->refreshInputs();
    }
}

bool ExecutionNode::connected() const {
    bool conn = true;
    for (auto din : datain) {
        conn = conn && din->connected();
    }
    return conn;
}

void ExecutionNode::execute() {

    for (auto din : datain) {
        din->execute();
    }

    for (auto dout : dataout) {
        dout->execute();
    }

}

std::string ExecutionNode::parameters() const {
    return "SpiNNaker node executing " + pynnscript;
}

bool ExecutionNode::isStarted() const {
    started_mutex.lock();
    bool sted = started;
    started_mutex.unlock();
    return sted;
}

void ExecutionNode::spikes_start(char* label, SpynnakerLiveSpikesConnection*) {
    if (sec::isVerbose())
        std::cerr << "[SpiNNaker::ExecutionNode] Starting simulation" << std::endl;
    started_mutex.lock();
    started = true;
    started_mutex.unlock();
}

bool ExecutionNode::processFinished() const {
    if (ended)
        return true;

    int status = 0;
    pid_t ret = waitpid(-1, &status, WNOHANG);
    if (ret == child && WIFEXITED(status)) {
        ended = true;
    }
    return ended;
}

}
