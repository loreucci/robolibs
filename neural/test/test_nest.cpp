#include "neural/nest/execnode.h"
#include "neural/nest/commondevices.h"
#include "neural/spikelogger.h"

#include <utilities/signals.h>
#include <sec/simplesources.h>
#include <sec/controller.h>
#include <sec/connections.h>


int main() {

    try {
        nest::ExecutionNode::initPythonRuntime();


        auto rah = Signals::rampandhold(100, 50, 2.0, 1.0, 100.0);
        sec::SignalSource ss(rah, 100.0);

        boost::python::list pgl;
        pgl.append(1);
        nest::PoissonGeneratorSetter pg(pgl);

        sec::connect(&ss.output, &pg.rate);

        boost::python::list sdl;
        sdl.append(2);
        nest::SpikeDetectorGetter sd(sdl);

        nest::ExecutionNode nn("test_nest.py", {&pg}, {&sd}, 100.0);
        nn.suppressOutput();
        nn.runOnSingleThread();

        neural::SpikeLogger logger("testspikes");
        sec::connect(&sd.spikes, logger);

        sec::main_controller.run(3.0);

        sd.generatePyplot();

    } catch (boost::python::error_already_set const &) {
        PyErr_Print();
        return 1;
    }

}
