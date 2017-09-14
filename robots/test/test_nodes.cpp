#include <iostream>

#include <sec/printer.h>
#include <sec/simplesources.h>
#include <sec/sec.h>
#include "utilities/signals.h"

#include "robots/icub/icubsim_utilities.h"
#include "robots/icub/icubrobot.h"
#include "robots/icub/icubsim.h"
#include "robots/icub/nodes.h"


int main(void) {

    sec::setSleeper(new iCubSimSleeper());

    iCubRobot<iCubSimHead> sim("icubSim");

    EncodersHeadVel he(sim, 100.0);
    sec::Printer printer("");
    sec::connect(he, &EncodersHeadVel::eyesversionvel, printer, " ");

    auto s = Signals::sin(20.0, 0.5, 0.0, 100.0);
    sec::SignalSource ss(s, 100.0);
    HeadVelocityControl mv(sim, 100.0);
    sec::connect(ss.output, mv.version);
//    sec::connect(ss.output, printer, " ");

    sec::main_controller.run();

    sim.head->movePosJoint(4, 0, true);

    return 0;

}
