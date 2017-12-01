#include "robots/icub/commons.h"

#include <utilities/signals.h>
#include <sec/simplesources.h>
#include <sec/synchronization.h>


int main(void) {

    auto s = Signals::sin(5.0, 2.0, 0.0, 100.0);
    sec::SignalSource ss(s, 100.0);

    VelocityObserver obs("obj/Vel", "signalder", 2.0, 10);

    for (unsigned int i = 0; i < 100; i++) {
        ss.execute();
        double x = ss.output.getData().first;
//        std::cout << x << std::endl;
        auto der = obs.derive({x});
        std::cout << x << " " << der[0] << std::endl;
        sec::synchronizer.sleep(10.0);
    }

}
