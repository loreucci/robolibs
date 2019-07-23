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

#ifndef DATALOGGER_H
#define DATALOGGER_H

#include <string>

#include "node.h"
#include "datalistener.h"
#include "resultscollector.h"


namespace sec {

class DataLogger : public Node, public Logger {

public:
    DataLogger(const std::string& separator = " ", const std::string& filename = "data.txt", double freq = 0.0);

    virtual ~DataLogger();

    virtual void refreshInputs() override;

    virtual bool connected() const override;

    virtual void execute() override;

    virtual std::string parameters() const override;

    void addListener(DataListener* l);

    template <typename T>
    void addListener(std::function<T(void)> fn, const std::string& name) {
        listeners.push_back(new FunctionListener<T>(name, fn));
    }

    void setFilename(const std::string& filename);
    void setSeparator(const std::string& separator);

    virtual bool logToFile() const override;

    virtual void reset() override;

    virtual void setPrefix(const std::string& prefix) override;

    void toggleHeaders(bool shouldsave = true);
    void toggleTime(bool shouldsave = true);

protected:
    std::string filename, separator, prefix;
    unsigned int counter;
    std::vector<DataListener*> listeners;
    bool headers, time;

    void toFile(std::ostream& o) const;

    void deleteall();

};

// connections
template <typename T>
void connect(NodeOut<T>& out, DataLogger& logger, const std::string& name) {

    DataListener* l = new NodeListener<T>(name, &out);

    logger.addListener(l);

}

template <typename T1, typename T2, typename F>
void connect(NodeOut<T1>& out, DataLogger& logger, const std::string& name, F fun) {

    DataListener* l = new NodeListener<T2>(name, new LinkFunction<T1, T2>(&out, fun));

    logger.addListener(l);

}

}

#endif // DATALOGGER_H
