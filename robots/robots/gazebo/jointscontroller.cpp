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

#include "jointscontroller.h"

#include <std_msgs/Float64.h>


JointsController::JointsController(const std::vector<std::string>& names, double freq)
    :sec::Node::Node(freq), names(names) {

    publishers.clear();

    for (const auto& nm : names) {

        publishers.push_back(n.advertise<std_msgs::Float64>(nm, 100));

    }

}

void JointsController::refreshInputs() {
    commands.refreshData();
}

bool JointsController::connected() const {
    return commands.isConnected();
}

void JointsController::execute() {


    if (commands.isNew()) {

        auto cmd = commands.getData();

        if (cmd.size() != publishers.size()) {
            std::cerr << "[JointsController] Wrong input size, not sending commands." << std::endl;
        }

        for (unsigned int i = 0; i < cmd.size(); i++) {
            std_msgs::Float64 msg;
            msg.data = cmd[i];
            publishers[i].publish(msg);
        }

    }


}

std::string JointsController::parameters() const {

    std::string ret = "Gazebo joints controller (";
    for (const auto& nm : names) {
        ret += nm + ", ";
    }
    ret.pop_back();
    ret.pop_back();
    ret += ")";

    return ret;

}
