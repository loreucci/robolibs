#include "robots/icub/icubsim_utilities.h"

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
    auto obj = world.createSphere(0.04, 0, 0.9, 0.5, 1, 0, 0);
    sec::synchronizer.sleep(50);

    // move object
    sec::SinusoidalSource ss(0.5, 1.0, 0.0, 0.0, 100.0);
    iCubSimObjectMover mover(world, obj, true);
    sec::connect(ss, &sec::SinusoidalSource::output, mover, &iCubSimObjectMover::x);

    sec::main_controller.run(5.0);

    return 0;
}
