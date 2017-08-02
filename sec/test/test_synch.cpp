#include "sec/sleeper.h"
#include "sec/sec.h"
#include "sec/plottingclient.h"
#include "sec/simplesources.h"
#include <utilities/signals.h>


int main(void) {

//    sec::Semaphore sem;
//    auto fun = [&sem]() {sem.wait(); sem.completion_notify();};
//    std::thread t{fun};
//    sem.wakeup();
//    sem.completion_wait();
//    t.join();

//    return 0;

    sec::setSleeper(new sec::Barrier());

    auto s = Signals::sin(10.0, 2.0, 1.0, 100.0);
    sec::SignalSource ss(s, 100.0);
    ss.setDelay(2.0);

    sec::PlottingClient plotter(100.0);
    plotter.runOnSingleThread();
    sec::connect(ss.output, plotter, "sin");

    sec::run(5.0);

    return 0;

}
