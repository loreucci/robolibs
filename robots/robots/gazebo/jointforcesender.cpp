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
