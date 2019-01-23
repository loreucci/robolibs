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

#ifndef GAZEBORUNNER_H
#define GAZEBORUNNER_H

#include <sec/node.h>

#include <ros/ros.h>

// This class needs the custom Gazebo plugins that can be found at:
// https://bitbucket.org/hbpneurorobotics/gazeborospackages
class GazeboRunner : public sec::Node {

public:
    GazeboRunner(double freq = 100.0);
    virtual ~GazeboRunner();

    virtual void refreshInputs() override;

    virtual bool connected() const override;

    virtual void execute() override;

    virtual std::string parameters() const override;

protected:
    ros::NodeHandle n;
    ros::ServiceClient advancesim;
    double timestep;

};

#endif // GAZEBORUNNER_H
