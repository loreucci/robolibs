#include "robots/icub/icubrobot.h"
#include "robots/icub/icubsim.h"

int main(void) {

    iCubRobot<iCubSimHead, iCubSimTorso, iCubSimInertial> sim("icubSim");

    std::cout << sim.parameters() << std::endl;

//    sim.head->movePos({0, -70, 0, 0, 0, 0}, true);
//    sim.head->movePos({0, 0, 0, 0, 0, 0}, true);

//    sim.head->movePosJoint(1, -70, true);
//    sim.head->movePosJoint(1, 0, true);

//    sim.torso->movePos({0, -20, 0}, true);
//    sim.torso->movePos({0, 0, 0}, true);

//    sim.torso->movePosJoint(1, -20, true);
//    sim.torso->movePosJoint(1, 0, true);

    return 0;

}
