#include "sec/socketdataexchange.h"

#include "sec/plottingclient.h"
#include "sec/sec.h"
#include "sec/simplesources.h"

#include <utilities/signals.h>

int main(void) {

    sec::SocketDataOut dataout("testdataexchange", 100.0);
    dataout.waitForServer();

    auto s = Signals::sin(2.0, 1.0, 0.0, 100.0);
    sec::SignalSource ss(s, 100.0);

    sec::connect(ss, &sec::SignalSource::output, dataout, "e1");

    dataout.sendStart();

    sec::run(5.0);

    return 0;

}
