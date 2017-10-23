#include "jointsreader.h"

#include <utilities/utilities.h>


JointsReader::JointsReader(const std::string& jointtopic, const std::vector<std::string>& joints, double freq)
    :sec::Source(freq), topic(topic), joints(joints) {

    sub = n.subscribe(jointtopic, 1000, &JointsReader::jointCallback, this);

    for (const auto& j : joints) {
        posrad.insert(std::make_pair(j, sec::NodeOut<double>()));
        velrad.insert(std::make_pair(j, sec::NodeOut<double>()));
        posdeg.insert(std::make_pair(j, sec::NodeOut<double>()));
        veldeg.insert(std::make_pair(j, sec::NodeOut<double>()));
    }

}

void JointsReader::execute() {
    ros::spinOnce();
//    ros::getGlobalCallbackQueue()->callAvailable(ros::WallDuration(0));
}

std::string JointsReader::parameters() const {
    return "Joints reader on " + topic;
}

void JointsReader::jointCallback(const sensor_msgs::JointState::ConstPtr& msg) {

    unsigned int sz = joints.size();
    Utils::Vector _allposrad(sz), _allvelrad(sz), _allposdeg(sz), _allveldeg(sz);

    for (unsigned int i = 0; i < msg->name.size(); i++) {

        auto n = std::find(joints.begin(), joints.end(), msg->name[i]);

        unsigned int idx = std::distance(joints.begin(), n);

        if (n != joints.end()) {

            posrad[msg->name[i]] = _allposrad[idx] = msg->position[i];
            velrad[msg->name[i]] = _allvelrad[idx] = msg->velocity[i];
            posdeg[msg->name[i]] = _allposdeg[idx] = Utils::radtodeg(msg->position[i]);
            veldeg[msg->name[i]] = _allveldeg[idx] = Utils::radtodeg(msg->velocity[i]);

        }

    }

    allposrad = _allposrad;
    allvelrad = _allvelrad;
    allposdeg = _allposdeg;
    allveldeg = _allveldeg;

}
