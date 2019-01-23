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

#include "sec/datalogger.h"
#include "sec/sec.h"
#include "sec/simplesources.h"


int main(void) {

    sec::setSleeper(new sec::NoSleeper());

    sec::setResultsName("res/prova");
    sec::setResultsMode(sec::SINGLE_FILES_MODE);

    auto s = Signals::sin(10.0, 2.0, 1.0, 100.0);
    sec::SignalSourceVector ss({s, s, s}, 100.0);
    ss.setDelay(2.0);

    sec::DataLogger logger;
    sec::connect(ss.output, logger, "a1 a2 a3");

    sec::saveAllNodesParameters();

    sec::setDefaultFrequency(100.0);

    sec::run(5.0);

    return 0;

}
