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

#ifndef CARTESIAN_H
#define CARTESIAN_H

#include "utilities/vector.h"

#include "sec/node.h"
#include "sec/nodelink.h"

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/CartesianControl.h>


class CartesianController : public sec::Node {

public:
    CartesianController(const std::string& robotName, const std::string& part, double freq = 0.0);
    ~CartesianController();

    virtual void refreshInputs() override;

    virtual bool connected() const override;

    virtual void execute() override;

    virtual std::string parameters() const override;

    void goToPose(const Utils::Vector& newpos, const Utils::Vector& newori, bool wait = false);

    std::pair<Utils::Vector, Utils::Vector> getPose() const;

    sec::NodeIn<Utils::Vector> inpos, inori;
    sec::NodeOut<Utils::Vector> currpos, currori;


protected:
    yarp::dev::PolyDriver clientCartCtrl;
    yarp::dev::ICartesianControl* icart;

    std::string part;

};

#endif // CARTESIAN_H
