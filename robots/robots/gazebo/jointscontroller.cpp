#include "jointscontroller.h"

#include <std_msgs/Float64.h>


JointsController::JointsController(const std::vector<std::string>& names, double freq)
    :sec::Node::Node(freq), names(names) {

    publishers.clear();

    for (const auto& nm : names) {

        publishers.push_back(n.advertise<std_msgs::Float64>(nm, 100));

    }

}

void JointsController::refreshInputs() {
    commands.refreshData();
}

bool JointsController::connected() const {
    return commands.isConnected();
}

void JointsController::execute() {


    if (commands.isNew()) {

        auto cmd = commands.getData();

        if (cmd.size() != publishers.size()) {
            std::cerr << "JointsController: wrong input size, not sending commands." << std::endl;
        }

        for (unsigned int i = 0; i < cmd.size(); i++) {
            std_msgs::Float64 msg;
            msg.data = cmd[i];
            publishers[i].publish(msg);
        }

    }


}

std::string JointsController::parameters() const {

    std::string ret = "Gazebo joints controller (";
    for (const auto& nm : names) {
        ret += nm + ", ";
    }
    ret.pop_back();
    ret.pop_back();
    ret += ")";

    return ret;

}
