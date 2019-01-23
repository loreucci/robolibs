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

#include "sec/launcherinterface.h"

#include "sec/plottingclient.h"
#include "sec/sec.h"
#include "sec/argumentparser.h"
#include "sec/simplesources.h"

#include <QCoreApplication>
#include <iostream>
#include <utilities/signals.h>

int main(int argc, char* argv[]) {

    QCoreApplication app(argc, argv);

    sec::addLauncherSocketArgument("testdataexchange");
    sec::argument_parser.parseArguments(argc, argv);
    sec::argument_parser.saveArgsToFile();

    auto s = Signals::sin(2.0, 1.0, 0.0, 100.0);
    sec::SignalSource ss(s, 100.0);

    sec::LauncherSocketDataIn datain("testdataexchange", {"e1", "e2"}, {2.0, 1.0}, 100.0);
    datain.addExpectedInput("e3", 4.0);

    sec::PlottingClient plotter(100.0);
    plotter.addConnection(&(datain.output("e1")), "e1", sec::identity_fun);
    plotter.addConnection(&(datain.output("e2")), "e2", sec::identity_fun);
    plotter.addConnection(&(datain.output("e3")), "e3", sec::identity_fun);

    datain.waitForStart();

    std::cout << "started" << std::endl;

    sec::run();

    return 0;

}
