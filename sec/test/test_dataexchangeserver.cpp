#include "sec/socketdataexchange.h"

#include "sec/plottingclient.h"
#include "sec/sec.h"
#include "sec/argumentparser.h"

#include <QCoreApplication>
#include <iostream>

int main(int argc, char* argv[]) {

    QCoreApplication app(argc, argv);

    sec::argument_parser.addArgument("defaultdatasocket", "testdataexchange");
    sec::argument_parser.parseArguments(argc, argv);
    sec::argument_parser.saveArgsToFile();

    sec::SocketDataIn datain("testdataexchange", {"e1", "e2"}, {2.0, 1.0}, 100.0);
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
