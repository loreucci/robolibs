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
