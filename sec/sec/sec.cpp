/*
 * Copyright (C) 2014-2019 Lorenzo Vannucci
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

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
