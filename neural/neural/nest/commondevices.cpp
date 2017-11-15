#include "commondevices.h"

#include "python_commons.h"

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

    py_exec("from nest import raster_plot");
    py_exec("raster_plot.from_device(%s)" % gids);
    py_exec("raster_plot.show()");

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

SpikeDetectorGetter createSpikeDetectorGetter(const boost::python::tuple& gids) {

    py::tuple d = py_eval<py::tuple>("nest.Create('spike_detector', 1)");
    py_exec("nest.Connect(%s, %s)" % py::make_tuple(gids, d));

    return SpikeDetectorGetter(d);

}

}
