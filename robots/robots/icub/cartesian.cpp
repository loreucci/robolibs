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

#include "cartesian.h"

#include "commons.h"

#include <iostream>


CartesianController::CartesianController(const std::string& robotName, const std::string& part, double freq)
    :sec::Node(freq), icart(nullptr), part(part) {

    yarp::os::Property options;
    options.put("device","cartesiancontrollerclient");
    options.put("remote","/" + robotName + "/cartesianController/" + part);
    options.put("local","/cartesianclient/" + part);

    if (!clientCartCtrl.open(options)) {
        throw iCubException("[CartesianController] Unable to create " + part + " device.");
    }

    if (clientCartCtrl.isValid()) {
       clientCartCtrl.view(icart);
    }

}

CartesianController::~CartesianController() {
    icart->stopControl();
    icart = nullptr;
    clientCartCtrl.close();
}

void CartesianController::refreshInputs() {
    inpos.refreshData();
    inori.refreshData();
}

bool CartesianController::connected() const {

    return inpos.isConnected() && inori.isConnected();

}

void CartesianController::execute() {

    goToPose(inpos.getData(), inori.getData(), false);
    auto currpose = getPose();
    currpos = currpose.first;
    currori = currpose.second;

}

std::string CartesianController::parameters() const {
    return "Cartesian controller for " + part;
}

void CartesianController::goToPose(const Utils::Vector& newpos, const Utils::Vector& newori, bool wait) {

    if (newpos.size() != 3 || newori.size() != 4) {
        std::cerr << "[CartesianController] Wrong size of request" << std::endl;
    }

    auto xd = iCubUtils::convert(newpos);
    auto od = iCubUtils::convert(newori);

    if (wait) {
        icart->goToPoseSync(xd,od);   // send request and wait for reply
        icart->waitMotionDone(0.04);  // wait until the motion is done and ping at each 0.04 seconds
    } else {
        icart->goToPose(xd, od);
    }

}

std::pair<Utils::Vector, Utils::Vector> CartesianController::getPose() const {
    yarp::sig::Vector x,o;
    icart->getPose(x, o);
    return std::make_pair(iCubUtils::convert(x), iCubUtils::convert(o));
}
