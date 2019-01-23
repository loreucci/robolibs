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

#include <sec/sec.h>

#include <utilities/signals.h>

#include <sec/simplesources.h>
#include <sec/resultscollector.h>
#include <sec/datalogger.h>


int main(void) {

    sec::setSleeper(new sec::NoSleeper());

    sec::setResultsMode(sec::SINGLE_FILES_MODE);
    sec::setResultsName("testtrials");

    sec::setDefaultFrequency(100.0);

    auto s = Signals::ramp(1.0, 0.0);
    sec::SignalSource ss(s);

    sec::DataLogger logger;
    sec::connect(ss.output, logger, "x");

    sec::runTrials(5, {}, {}, 2.0, {});

    return 0;
}
