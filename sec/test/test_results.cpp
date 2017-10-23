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

    sec::run(5.0);

    return 0;

}
