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

#ifndef SPIKELOGGER_H
#define SPIKELOGGER_H

#include <sec/node.h>
#include <sec/resultscollector.h>

#include "spikes.h"


namespace neural {


class SpikeLogger : public sec::Node, public sec::Logger {

public:
    SpikeLogger(const std::string& filename = "spikes.txt", double freq = 0.0);

    virtual ~SpikeLogger();

    virtual void refreshInputs() override;

    virtual bool connected() const override;

    virtual void execute() override;

    virtual std::string parameters() const override;

    void addSpikeSource(SpikeNodeOut* source);

    virtual bool logToFile() const override;

    virtual void reset() override;

    virtual void setPrefix(const std::string& prefix) override;

protected:
    std::string filename, prefix;
    std::vector<SpikeNodeIn*> listeners;

    SpikeData data;

};


}

namespace sec {

void connect(neural::SpikeNodeOut& out, neural::SpikeLogger& sink);

}

#endif // SPIKELOGGER_H
