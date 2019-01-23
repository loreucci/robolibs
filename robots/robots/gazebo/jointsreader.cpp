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

#include "jointsreader.h"

#include <utilities/utilities.h>


JointsReader::JointsReader(const std::string& jointtopic, const std::vector<std::string>& joints, double freq)
    :sec::Source(freq), topic(jointtopic), joints(joints) {

    sub = n.subscribe(jointtopic, 1000, &JointsReader::jointCallback, this);

    for (const auto& j : joints) {
        posrad.insert(std::make_pair(j, sec::NodeOut<double>()));
        velrad.insert(std::make_pair(j, sec::NodeOut<double>()));
        posdeg.insert(std::make_pair(j, sec::NodeOut<double>()));
        veldeg.insert(std::make_pair(j, sec::NodeOut<double>()));
    }

}

void JointsReader::execute() {
    ros::spinOnce();
//    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0));
}

std::string JointsReader::parameters() const {
    return "Joints reader on " + topic;
}

void JointsReader::jointCallback(const sensor_msgs::JointState::ConstPtr& msg) {

    unsigned int sz = joints.size();
    Utils::Vector _allposrad(sz), _allvelrad(sz), _allposdeg(sz), _allveldeg(sz);

    for (unsigned int i = 0; i < msg->name.size(); i++) {

        auto n = std::find(joints.begin(), joints.end(), msg->name[i]);

        unsigned int idx = std::distance(joints.begin(), n);

        if (n != joints.end()) {

            posrad[msg->name[i]] = _allposrad[idx] = msg->position[i];
            velrad[msg->name[i]] = _allvelrad[idx] = msg->velocity[i];
            posdeg[msg->name[i]] = _allposdeg[idx] = Utils::radtodeg(msg->position[i]);
            veldeg[msg->name[i]] = _allveldeg[idx] = Utils::radtodeg(msg->velocity[i]);

        }

    }

    allposrad = _allposrad;
    allvelrad = _allvelrad;
    allposdeg = _allposdeg;
    allveldeg = _allveldeg;

}
