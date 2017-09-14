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
