#ifndef MAINTHREAD_H
#define MAINTHREAD_H

#include <vector>
#include <functional>

#include "synchronization.h"
#include "controller.h"
#include "source.h"
#include "experimentlogger.h"


template <typename A, typename B, typename R>
class MainThread {

public:
    MainThread(Source<A>& source, Controller<A, B>& ctrl, R& robot,
               std::vector<std::function<bool(void)>> terminate = {},
               std::vector<std::function<void(void)>> pre = {},
               std::vector<std::function<void(void)>> post = {})
        :source(source), ctrl(ctrl), robot(robot), terminate(terminate), pre(pre), post(post) {

        // set same frequency for controller and source
        source.setSamplingFreq(ctrl.getFrequency());

        // register semaphore
        if (ctrl.getFrequency() == 0.0) {
            wait = false;
//            throw std::invalid_argument("MainThread: controller has no specified frequency.");
        } else {
            wait = true;
            s = synchronizer.registerSignal(ctrl.getFrequency());
        }

        // log results by default
        log = true;

        // loop
        loop = true;

    }

    void disableLogging() {
        log = false;
        explogger.disableLogging();
    }

    void run(unsigned int steps = 0) {

        if (wait) {
            synchronizer.start();
        }

        unsigned int count = 0;

        while (loop) {

            if (wait) {
                s.wait();
            }

            // pre functions
            for (auto& p : pre) {
                p();
            }

            // actual control
            robot.move(ctrl.controlAndGetOutput(source()));

            // post functions
            for (auto& p : post) {
                p();
            }   

            // logs
            if (log) {
                explogger.readValues();
            }

            // exits for external termination conditions
            for (auto& e : terminate) {
                if (e())
                    loop = false;
            }

            // exits if there are no more inputs
            if (source.ended())
                break;

            // exits if max number of steps is reached
            if (steps > 0 && count >= steps)
                break;
            count++;

        }

        synchronizer.quitAll();

        explogger.saveToFile();

    }


protected:
    Source<A>& source;
    Controller<A, B>& ctrl;
    R& robot;
    Semaphore s;
    bool log, loop, wait;
    std::vector<std::function<bool(void)>> terminate;
    std::vector<std::function<void(void)>> pre, post;

};

#endif // MAINTHREAD_H
