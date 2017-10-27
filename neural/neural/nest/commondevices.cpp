#include "commondevices.h"

#include <boost/python.hpp>
namespace py = boost::python;


namespace nest {

bool PoissonGeneratorSetter::connected() const {
    return rate.isConnected();
}

void PoissonGeneratorSetter::refreshInputs() {
    rate.refreshData();
}

void PoissonGeneratorSetter::generateParams() {
    params["rate"] = rate.getData();
}


void SpikeDetectorGetter::generatePyplot() {

    py::exec("from nest import raster_plot", main_namespace);
    py::object cmd = "raster_plot.from_device(%s)" % gids;
    std::string cmd_str = py::extract<std::string>(cmd);  // this may be needed because of a bug of boost 1.65
    py::exec(cmd_str.c_str(), main_namespace);
    py::exec("raster_plot.show()", main_namespace);

}

void SpikeDetectorGetter::consumeParams() {

    py::object ids = params["events"]["senders"];
    py::object times = py::extract<py::object>(params["events"]["times"]);

    neural::SpikeData sp;
    unsigned int len = py::len(ids);
    for (int i = lastlen; i < len; i++) {
        unsigned int id = py::extract<unsigned int>(ids[i]);
        double time = py::extract<double>(times[i]);
        sp.push_back({id, time/1000.0});
    }

    lastlen = len;
    spikes.addData(sp);

}

}
