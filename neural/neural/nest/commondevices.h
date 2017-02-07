#ifndef NESTCOMMONDEVICES_H
#define NESTCOMMONDEVICES_H

#include <sec/nodelink.h>

#include "../spikes.h"
#include "datainterfaces.h"


namespace nest {

class PoissonGeneratorSetter : public StatusSetter {

public:
    using StatusSetter::StatusSetter;

    virtual bool connected() const;

    virtual void refreshInputs();

    sec::NodeIn<double> rate;

protected:
    virtual void generateParams() override;

};


class SpikeDetectorGetter : public StatusGetter {

public:
    SpikeDetectorGetter(const boost::python::list& gids, double timestep = 10.0);

    void generatePyplot();

    neural::SpikeNodeOut spikes;

protected:
    unsigned lastlen;
    virtual void consumeParams() override;

};

}

#endif // NESTCOMMONDEVICES_H
