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
