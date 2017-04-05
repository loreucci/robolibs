#include "sec/datalogger.h"
#include "sec/sec.h"
#include "sec/simplesources.h"


int main(void) {

    sec::setSleeper(new sec::NoSleeper());

    sec::setResultsName("prova");
    sec::setResultsMode(sec::ResultsCollector::SINGLE_FILES_MODE);

    auto s = Signals::sin(10.0, 2.0, 1.0, 100.0);
    sec::SignalSource ss(s, 100.0);

    sec::DataLogger logger;
    sec::connect(ss, &sec::SignalSource::output, logger, "y");

    sec::saveAllNodesParameters();

    sec::run(10.0);

    return 0;

}
