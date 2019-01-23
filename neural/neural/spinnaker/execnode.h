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

#ifndef SPINNAKEREXECNODE_H
#define SPINNAKEREXECNODE_H

#include <string>
#include <vector>

#include <sec/node.h>

#include <SpynnakerLiveSpikesConnection.h>

#include "datainterfaces.h"


namespace SpiNNaker {

class ExecutionNode : public sec::Node, SpikesStartCallbackInterface {

public:
    ExecutionNode(const std::string& pynnscript, std::vector<DataInjector*> datain, std::vector<DataReceiver*> dataout, double freq = 100.0);

    virtual ~ExecutionNode();

    virtual void refreshInputs() override;

    virtual bool connected() const override;

    virtual void execute() override;

    virtual std::string parameters() const override;

    // synchronization with main controller
    virtual void spikes_start(char*, SpynnakerLiveSpikesConnection*) override;
    bool isStarted() const;
    bool processFinished() const;

protected:
    const std::string& pynnscript;
    std::vector<DataInjector*> datain;
    std::vector<DataReceiver*> dataout;

    std::vector<char*> send_labels, receive_labels;

    SpynnakerLiveSpikesConnection* connection;

    bool started = false;
    mutable std::mutex started_mutex;
    mutable bool ended = false;

    pid_t child;

};

}

#endif // SPINNAKEREXECNODE_H
