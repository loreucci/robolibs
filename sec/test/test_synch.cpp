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
