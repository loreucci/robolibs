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

#ifndef JOINTSCONTROLLER_H
#define JOINTSCONTROLLER_H

#include <vector>
#include <string>

#include <ros/ros.h>

#include <utilities/vector.h>
#include <sec/node.h>
#include <sec/nodelink.h>


class JointsController : public sec::Node {

public:
    JointsController(const std::vector<std::string>& names, double freq = 100.0);

    virtual void refreshInputs() override;

    virtual bool connected() const override;

    virtual void execute() override;

    virtual std::string parameters() const override;

    sec::NodeIn<Utils::Vector> commands;

protected:
    ros::NodeHandle n;
    std::vector<ros::Publisher> publishers;
    std::vector<std::string> names;
};

#endif // JOINTSCONTROLLER_H
