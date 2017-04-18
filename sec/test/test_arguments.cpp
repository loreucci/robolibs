#include <iostream>
#include <string>

#include <utilities/signals.h>

#include "sec/sec.h"
#include "sec/simplesources.h"
#include "sec/argumentparser.h"
#include "sec/printer.h"


int main(int argc, char* argv[]) {

//    // Any
//    bool c = false;
//    std::string p = "prova";
//    sec::Any a("prova");

//    std::cout << a.getValue<std::string>() << std::endl;

//    bool b = a.getValue<bool>();
////    double b = a.getValue<double>();
//    std::cout << b << std::endl;


    sec::argument_parser.addArgument("duration", 2.0);

    sec::argument_parser.addArgument("prova23", "ciao");

//    sec::argument_parser.printArgs();

//    std::cout << sec::argument_parser.getValue<double>("prova") << std::endl;

//    std::cout << "\n\narguments:\n";
    sec::argument_parser.parseArguments(argc, argv);

//    std::cout << "\n\n";
//    sec::argument_parser.printArgs();

    sec::argument_parser.saveArgsToFile();

    auto s = Signals::sin(10.0, 1.0, 0.0, 100.0);
    sec::SignalSource ss(s, 100.0);

    sec::Printer printer(" ", 100.0);
    sec::connect(ss, &sec::SignalSource::output, printer, " ");

    sec::run(sec::getArgumentValue<double>("duration"));

    return 0;

}
