#include "jointsreader.h"

#include <utilities/utilities.h>


JointsReader::JointsReader(double freq)
    :sec::Source(freq) {

    sub = n.subscribe("/iCub/joints", 1000, &JointsReader::jointCallback, this);

}

void JointsReader::execute() {
    ros::spinOnce();
//    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0));
}

std::string JointsReader::parameters() const {
    return "Joints Reader";
}

void JointsReader::jointCallback(const sensor_msgs::JointState::ConstPtr& msg) {

//    std::cout << "callback\n";

    for (unsigned int i = 0; i < msg->name.size(); i++) {
        if (msg->name[i] == "r_elbow") {
            posrad = msg->position[i];
            velrad = msg->velocity[i];
            posdeg = Utils::radtodeg(msg->position[i]);
            veldeg = Utils::radtodeg(msg->velocity[i]);
        }
    }

}
