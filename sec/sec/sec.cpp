#include "sec.h"

#include <iostream>

#include "resultscollector.h"


namespace sec {

void run(double time, std::vector<std::function<bool(void)>> endconditions) {
    main_controller.run(time, endconditions);
}

void sleep(double ms) {
    synchronizer.sleep(ms);
}

void setSleeper(Sleeper* sleeper) {
    synchronizer.setSleeper(sleeper);
}

void runTrials(unsigned int N, std::vector<std::function<void(void)>> pre, std::vector<std::function<void(void)>> post, double time, std::vector<std::function<bool(void)>> endconditions) {

    // notify results collector
    results_collector.setTrials(N);

    for (unsigned int i = 0; i < N; i++) {

        if (sec::isVerbose())
            std::cerr << "[Controller] Running trial " << i << " of " << N << std::endl;

        // reset nodes
        if (i > 0)
            main_controller.resetAllNodes();

        // reset results
        results_collector.setCurrentTrial(i+1);

        // pre functions
        for (auto p : pre)
            p();

        // actual trial
        run(time, endconditions);

        // post functions
        for (auto p : post)
            p();

    }

}

}
