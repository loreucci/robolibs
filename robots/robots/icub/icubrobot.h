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

#ifndef ICUBROBOT_H
#define ICUBROBOT_H

#include <string>
#include <vector>
#include <sec/source.h>

#include <utilities/utilities.h>

extern unsigned int iCubRobotInstanceCount;

template <class ... Parts>
class iCubRobot : public sec::Source, public Parts... {

public:
    iCubRobot(const std::string& robotname, double encoderFreq = 100.0)
        :sec::Source(encoderFreq), robotname(robotname) {

        localname = "robot" + std::to_string(iCubRobotInstanceCount);
        iCubRobotInstanceCount++;

        activateAll();

    }

    ~iCubRobot() {
        using expander = int[];
        expander{0, (Parts::deactivate(), 0)... };
    }

    void activateAll() {
        // some black magic from
        // https://stackoverflow.com/questions/30563254/how-can-i-expand-call-to-variadic-template-base-classes
        // in C++17, it should be:
        // (Parts::activate(), ...);
        // http://en.cppreference.com/w/cpp/language/fold

        using expander = int[];
        expander{0, (Parts::activate(robotname, localname), 0)... };
    }

    void refreshEncoders() {
        using expander = int[];
        expander{0, (Parts::refresh(), 0)... };
    }

    virtual void execute() override {
        refreshEncoders();
    }

    virtual std::string parameters() const override {
        std::vector<std::string> list{Parts::name()...};
        return "iCubRobot with: " + Utils::make_string(list, ", ");
    }

    std::string getLocalName() const {
        return localname;
    }

protected:
    std::string robotname, localname;

};

#endif // ICUBROBOT_H
