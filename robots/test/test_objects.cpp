#include "robots/icub/icubsim_utilities.h"

#include <utilities/signals.h>
#include <sec/synchronization.h>
#include <sec/controller.h>
#include <sec/simplesources.h>
#include <sec/connections.h>


int main(void) {

    sec::synchronizer.setSleeper(new iCubSimSleeper());

    iCubSimWorld world;
    world.clearAll();
    sec::synchronizer.sleep(50);

    // create object
    auto obj1 = world.createSphere(0.04, 0, 0.9, 0.5, 1, 0, 0);
    sec::synchronizer.sleep(50);
    auto obj2 = world.createBox(0.04, 0.04, 0.04, 0, 0.7, 0.5, 0, 1, 0);
    sec::synchronizer.sleep(50);
    auto obj3 = world.createCylinder(0.04, 0.04, 0, 1.1, 0.5, 0, 0, 1);
    sec::synchronizer.sleep(50);

    // move object
    auto s = Signals::sin(0.5, 1.0, 0.0, 100.0);
    sec::SignalSource ss(s, 100.0);
    iCubSimObjectMover mover1(world, obj1, true, 100.0);
    iCubSimObjectMover mover2(world, obj2, true, 100.0);
    iCubSimObjectMover mover3(world, obj3, true, 100.0);
    sec::connect(ss.output, mover1.x);
    sec::connect(ss.output, mover2.x);
    sec::connect(ss.output, mover3.x);

    sec::main_controller.run(5.0);

    return 0;
}
