#include "neural/nest/execnode.h"
#include "neural/nest/commondevices.h"
#include "neural/spikelogger.h"

#include <utilities/signals.h>
#include <sec/simplesources.h>
#include <sec/controller.h>
#include <sec/connections.h>
#include <sec/sec.h>


int main() {

    try {

        auto rah = Signals::rampandhold(100, 50, 2.0, 1.0, 100.0);
        sec::SignalSource ss(rah);

        nest::PoissonGeneratorSetter pg({1});
        sec::connect(ss.output, pg.rate);

        nest::ExecutionNode nn("test_nest.py", {&pg}, {});
        nn.suppressOutput();
        nn.runOnSingleThread();

        nest::SpikeDetectorGetter sd = nest::createSpikeDetectorGetter(nn.getPopulationGIDs("pg"));
//        nest::SpikeDetectorGetter sd(nn.getPopulationGIDs("sd"));
        nn.addDataOut({&sd});

//        neural::SpikeLogger logger;
//        sec::connect(sd.spikes, logger);

        sec::setDefaultFrequency(100.0);
        sec::main_controller.run(3.0);

        sd.generatePyplot();

    } catch (boost::python::error_already_set const &) {
        PyErr_Print();
        return 1;
    }

}
