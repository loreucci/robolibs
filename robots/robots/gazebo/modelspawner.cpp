#include "modelspawner.h"

#include <iostream>
#include <fstream>
#include <sstream>
#include <algorithm>

#include <gazebo_msgs/SpawnEntity.h>
#include <gazebo_msgs/DeleteModel.h>


ModelSpawner::ModelSpawner() {

    spawn = n.serviceClient<gazebo_msgs::SpawnEntity>("/gazebo/spawn_sdf_entity");
    del = n.serviceClient<gazebo_msgs::DeleteModel>("/gazebo/delete_model");

}

bool ModelSpawner::spawnModel(const std::string& name, const std::string& filename, const ModelSpawner::Position& pos, const ModelSpawner::Orientation& ori) {

    // read file
    std::ifstream file(filename);
    if (file) {

        std::stringstream contents;
        contents << file.rdbuf();
        file.close();

        // set request
        gazebo_msgs::SpawnEntity sp;
        sp.request.entity_name = name;
        sp.request.entity_xml = contents.str();
        sp.request.initial_pose.position.x = pos.x;
        sp.request.initial_pose.position.y = pos.y;
        sp.request.initial_pose.position.z = pos.z;
        sp.request.initial_pose.orientation.x = ori.x;
        sp.request.initial_pose.orientation.y = ori.y;
        sp.request.initial_pose.orientation.z = ori.z;
        sp.request.initial_pose.orientation.w = ori.w;

        spawn.call(sp);

        if (!sp.response.success ) {
            std::cerr << "GazeboSpawner: unable to spawn model " << filename << std::endl;
            std::cerr << "\t" << sp.response.status_message << std::endl;
            return false;
        }

    } else {
        std::cerr << "GazeboSpawner: unable to open file " << filename << std::endl;
        return false;
    }

    models.push_back(name);
    return true;

}

void ModelSpawner::clearAll() {

    for (auto& m : models) {
        deleteOnly(m);
    }

    models.clear();

}

bool ModelSpawner::deleteModel(const std::string& name) {

    if (!deleteOnly(name))
        return false;

    auto pos = std::remove_if(models.begin(), models.end(), [name](const std::string& m){return m == name;});
    models.erase(pos, models.end());

    return true;

}

bool ModelSpawner::deleteOnly(const std::string& name) {

    gazebo_msgs::DeleteModel d;
    d.request.model_name = name;

    del.call(d);

    if (!d.response.success) {
        std::cerr << "GazeboSpawner: unable to delete model " << name << std::endl;
        std::cerr << "\t" << d.response.status_message << std::endl;
        return false;
    }

    return true;

}
