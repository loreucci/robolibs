#include <robots/icub/icubsim.h>
#include <robots/icub/icubrobot.h>
#include <robots/icub/cartesian.h>
#include <sec/sec.h>

#include <yarp/dev/IControlMode.h>


int main (void) {

    sec::setDefaultFrequency(100.0);

    iCubRobot<iCubSimLeftArm> sim("icubSim");

    // home arm
    sim.leftarm->setControlMode(VOCAB_CM_POSITION);
//    sim.leftarm->home();
    sim.leftarm->movePos({   -56.7,   20,    0,   100,    85,    -20,    0,   60,   20,   20,   20,   10,   10,   10,   10,   10});

    CartesianController cart("icubSim", "left_arm");

    std::cout << cart.getPose().first << std::endl;
    std::cout << cart.getPose().second << std::endl;

    cart.goToPose({-0.29, -0.16, 0.2}, cart.getPose().second, true);

    return 0;

}
