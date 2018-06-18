#include "sec/sec.h"
#include "sec/printer.h"

int main(void) {

    sec::setDefaultFrequency(100.0);

    sec::AbsoluteClock clock("%H:%M:%S");

    sec::Printer printer;

    sec::connect(clock.formatted, printer);
    sec::connect(clock.epoch, printer);

    sec::run(10.0);

}
