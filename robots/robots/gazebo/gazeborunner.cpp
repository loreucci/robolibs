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

#include "gazeborunner.h"

#include <gazebo_msgs/AdvanceSimulation.h>
#include <std_srvs/Empty.h>
#include <gazebo_msgs/GetPhysicsProperties.h>

#include <cmath>

GazeboRunner::GazeboRunner(double freq)
    :sec::Node(freq) {

    ros::ServiceClient physics = n.serviceClient<gazebo_msgs::GetPhysicsProperties>("/gazebo/get_physics_properties");
    gazebo_msgs::GetPhysicsProperties phy;
    physics.call(phy);
    timestep = phy.response.time_step;

//    std::cout << timestep << std::endl;

    ros::ServiceClient pausesim = n.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
    std_srvs::Empty emptyreq;
    pausesim.call(emptyreq);


    advancesim = n.serviceClient<gazebo_msgs::AdvanceSimulation>("/gazebo/advance_simulation");

}

GazeboRunner::~GazeboRunner() {
    ros::ServiceClient unpausesim = n.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
    std_srvs::Empty emptyreq;
    unpausesim.call(emptyreq);
}

void GazeboRunner::refreshInputs() {}

bool GazeboRunner::connected() const {
    return true;
}

void GazeboRunner::execute() {

    int steps = std::round((1.0/freq) / timestep);

    gazebo_msgs::AdvanceSimulation adv;
    adv.request.steps = steps;
    advancesim.call(adv);


}

std::string GazeboRunner::parameters() const {
    return "Gazebo runner";
}
