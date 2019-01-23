#include <iostream>

#include <sec/printer.h>
#include <sec/simplesources.h>
#include <sec/sec.h>
#include "utilities/signals.h"

#include "robots/icub/icubsim_utilities.h"
#include "robots/icub/icubrobot.h"
#include "robots/icub/icubsim.h"
#include "robots/icub/nodes.h"

#include <yarp/dev/IControlMode.h>


int main(void) {

    sec::setSleeper(new iCubSimSleeper());

    iCubRobot<iCubSimLeftArm> sim("icubSim");

    sim.leftarm->setControlMode(VOCAB_CM_POSITION);
    sim.leftarm->movePos({   -56.7,   20,    0,   100,    85,    -20,    0,   60,   20,   20,   20,   10,   10,   10,   10,   10});
    //sim.leftarm->movePos({   -56.7,   35.37,    25.74,   95,    85,    -20,    0,   60,   20,   20,   20,   10,   10,   10,   10,   10});
    //sim.leftarm->movePos({   -56.7,   11.256,    -17.55,   105,    85,    -20,    0,   60,   20,   20,   20,   10,   10,   10,   10,   10});

//    sim.head->setControlMode(VOCAB_CM_VELOCITY);

//    sim.head->moveVel({0, 0, 0, 0, 0, 0});
//    sim.head->moveVelJoint(4, 0.0);


//    EncodersHeadVel he(sim, 100.0);
//    sec::Printer printer("", 100.0);
//    sec::connect(he.eyesversionvel, printer, " ");

    auto sr = Signals::sin(12.057, 0.4, 0.0, 100.0)+23.313;
    sec::SignalSource ssr(sr, 100.0);
    auto sy = Signals::sin(21.645, 0.4, 0.0, 100.0)+4.095;
    sec::SignalSource ssy(sy, 100.0);
    auto se = Signals::sin(5, 0.4, 3.14, 100.0)-100;
    sec::SignalSource sse(se, 100.0);
    LeftArmPositionControl mv(sim, 100.0);
    sec::connect(ssr.output, mv.shoulder_roll);
    sec::connect(ssy.output, mv.shoulder_yaw);
    sec::connect(ssy.output, mv.elbow);
//    sec::connect(ss.output, printer, " ");

    sec::main_controller.run();

//    sim.head->setControlMode(VOCAB_CM_POSITION);
//    sim.head->movePosJoint(4, 0, true);

//    std::cout << "fatto" << std::endl;

    return 0;

}
