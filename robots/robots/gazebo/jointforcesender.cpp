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

#include "jointforcesender.h"

#include <gazebo_msgs/ApplyJointEffort.h>
#include <cmath>

JointForceSender::JointForceSender(const std::string& jointname, double freq)
    :sec::Node(freq), jointname(jointname) {


    client = n.serviceClient<gazebo_msgs::ApplyJointEffort>("/gazebo/apply_joint_effort");

}

void JointForceSender::refreshInputs() {
    input.refreshData();
}

bool JointForceSender::connected() const {
    return input.isConnected();
}

void JointForceSender::execute() {

    gazebo_msgs::ApplyJointEffort srv;
    srv.request.joint_name = "r_elbow";
    srv.request.effort = input.getData();
//    int32_t nsec = std::ceil(1.0/getFrequency());
    srv.request.duration = ros::Duration(1.0/getFrequency());

    client.call(srv);

//    std::cout << "command sent\n";

}

std::string JointForceSender::parameters() const {
    return "JointForceSender for " + jointname;
}
