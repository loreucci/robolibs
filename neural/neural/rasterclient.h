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

#ifndef RASTERCLIENT_H
#define RASTERCLIENT_H

#include <vector>
#include <utility>

#include <sec/node.h>
#include <sec/nodelink.h>
#include <sec/controller.h>
#include <sec/qtclientserverbase.h>

#include "spikes.h"


namespace neural {

class RasterClient : public sec::QtClientBase, public sec::Node {

    Q_OBJECT

public:
    explicit RasterClient(double freq = 0.0, QObject *parent = 0);
    virtual ~RasterClient();

    void addConnection(SpikeNodeOut* out, const std::string& name);

    void clear();

    virtual void refreshInputs();

    virtual bool connected() const;

    virtual void execute();

    virtual std::string parameters() const;

protected:
    std::vector<std::pair<unsigned int, SpikeNodeIn*>> inputs;

    void changeFreq(double freq);

    void advance();

//    void sendSerialized(unsigned int id, const SpikeData& data);

    QString serialize(unsigned int id, const SpikeData& data);

};

}

namespace sec {

void connect(neural::SpikeNodeOut& out, neural::RasterClient& sink, const std::string& name);

}

#endif // RASTERCLIENT_H
