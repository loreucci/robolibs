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

#ifndef GAZEBOSPAWNER_H
#define GAZEBOSPAWNER_H

#include <string>
#include <vector>

#include <ros/ros.h>


class ModelSpawner {

public:

    struct Position {
        double x, y, z;
    };

    struct Orientation {
        double x, y, z, w;
    };


    ModelSpawner();

    bool spawnModel(const std::string& name, const std::string& filename, const Position& pos, const Orientation& ori);

    void clearAll();

    bool deleteModel(const std::string& name);

private:
    ros::NodeHandle n;
    ros::ServiceClient spawn, del;
    std::vector<std::string> models;

    bool deleteOnly(const std::string& name);

};

#endif // GAZEBOSPAWNER_H
