#include <robots/gazebo/jointscontroller.h>

#include <utilities/signals.h>
#include <sec/simplesources.h>
#include <sec/sec.h>

#include <ros/ros.h>


int main(int argc, char *argv[]) {

    ros::init(argc, argv, "test_jointscontroller");

    sec::SignalSourceVector sv({Signals::sin(1.0, 0.5, 0.0, 100.0), Signals::sin(1.0, 0.5, Utils::PI, 100.0)}, 100.0);

    JointsController ctrl({"/iCub/r_elbow/pos", "/iCub/l_elbow/pos"}, 100.0);

    sec::connect(sv.output, ctrl.commands);

    sec::run(5.0);

    return 0;
}
