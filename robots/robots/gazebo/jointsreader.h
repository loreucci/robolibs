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

#ifndef JOINTSREADER_H
#define JOINTSREADER_H

#include <unordered_map>

#include <utilities/vector.h>

#include <sec/simplesources.h>
#include <sec/nodelink.h>

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>


class JointsReader : public sec::Source {

public:
    JointsReader(const std::string& jointtopic, const std::vector<std::string>& joints, double freq = 100.0);

    virtual void execute() override;

    virtual std::string parameters() const override;

    std::unordered_map<std::string, sec::NodeOut<double>> posrad, velrad;
    std::unordered_map<std::string, sec::NodeOut<double>> posdeg, veldeg;

    sec::NodeOut<Utils::Vector> allposrad, allvelrad;
    sec::NodeOut<Utils::Vector> allposdeg, allveldeg;

protected:
    ros::NodeHandle n;
    ros::Subscriber sub;
    std::string topic;
    std::vector<std::string> joints;

    void jointCallback(const sensor_msgs::JointState::ConstPtr& msg);

};

#endif // JOINTSREADER_H
