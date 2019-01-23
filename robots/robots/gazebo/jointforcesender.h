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

#ifndef JOINTFORCESENDER_H
#define JOINTFORCESENDER_H

#include <string>

#include <sec/node.h>
#include <sec/nodelink.h>

#include <ros/ros.h>


class JointForceSender : public sec::Node {

public:
    JointForceSender(const std::string& jointname, double freq = 100.0);

    virtual void refreshInputs() override;

    virtual bool connected() const override;

    virtual void execute() override;

    virtual std::string parameters() const override;

    sec::NodeIn<double> input;

protected:
    std::string jointname;
    ros::NodeHandle n;
    ros::ServiceClient client;

};

#endif // JOINTFORCESENDER_H
