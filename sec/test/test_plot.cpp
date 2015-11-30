#include <sec/plottingclient.h>
#include <sec/controller.h>
#include <sec/simplesources.h>
#include <sec/logger.h>
#include <sec/connections.h>


int main(void) {

    sec::SinusoidalSource source(10.0, 1.0, 0.0, 0.0, 30.0);
    sec::PlottingClient plots(30.0);

    sec::SinusoidalSource source2(5.0, 2.0, 0.0, 5.0, 30.0);

//    sec::Logger logger;

    sec::connect(source2, &sec::SinusoidalSource::output, plots, "test2");
    sec::connect(source, &sec::SinusoidalSource::output, plots, "test");

//    sec::connect(source, &sec::SinusoidalSource::output, logger, "test");
//    sec::connect(source2, &sec::SinusoidalSource::output, logger, "test2");


    sec::main_controller.run(10.0);

}
